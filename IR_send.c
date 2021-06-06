
#ifndef IR_SEND_C_
#define IR_SEND_C_

#include "app/framework/include/af.h"

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_timer.h"
#include "dmadrv.h"

#include "IR_send.h"


#define CARRIER_FREQUENCY 38000
#define PRS_CH 2
#define IR_SEND_DEBUG
#define DUMMY_VALUE 1000

uint8_t count = 0;
uint32_t timer1Freq = 0;

int IR_frame[] = {DUMMY_VALUE, 560, 200, 560, 1600, 400, 150, 800, 3000, 40, DUMMY_VALUE};
const int length_IR_frame = sizeof (IR_frame) / sizeof (int);


void ir_frame_setup()
{
   if ((length_IR_frame % 2) == 0)
   {
      emberAfCorePrintln("Number of input symbol should be a odd number");
      abort();
   }

   uint64_t v;
   for (uint32_t i = 0; i < length_IR_frame; i++)
   {
      v = ((uint64_t)IR_frame[i] * (uint64_t)timer1Freq)/1000000;

#ifdef IR_SEND_DEBUG
      emberAfCorePrintln("v[%d] = %ld", i, v);
#endif

      if ((i != length_IR_frame - 1) && (v == 0 || (v >> 16)))
      {
          emberAfCorePrintln("Bad value %ld, %d", v, i);
          abort();
      }
      IR_frame[i] = v & 0xFFFF;
   }
   /* Set last dummy value to maximum */
   IR_frame[length_IR_frame - 1] = 0xFFFF;
}


void ir_init_topB(void)
{
  TIMER_TopSet(TIMER1, IR_frame[0]);
  TIMER_TopBufSet(TIMER1, IR_frame[1]);
}
/**************************************************************************//**
 * @brief
 *    TIMER 1 handler
 *****************************************************************************/

void TIMER1_IRQHandler(void)
{
  /*Check the interrupt flag */
  uint32_t flags = TIMER_IntGet(TIMER1);
  TIMER_IntClear(TIMER1, flags);

  if ((flags & TIMER_IF_OF) == 0)
    return;

  if ((TIMER1->STATUS & TIMER_STATUS_RUNNING) == 0)
     return;

  if (++count == length_IR_frame - 1)
  /* End of transfer */
  {
     TIMER_Enable(TIMER1,false);
     TIMER_Enable(TIMER2,false);
     GPIO_PinOutClear(SEND_PORT_BRD4182A, SEND_PIN_BRD4182A);
  }
}


/**************************************************************************//**
 * @brief
 *    IR generate 38kHz with a certain duty cycle with TIMER 2 CC0
 *****************************************************************************/

static volatile float dutyCycle = 0;

void IR_generate_freq(void)
{
  uint32_t topValue;
  uint32_t timer2Freq = 0;
  CMU_ClockEnable(cmuClock_TIMER2, true);

  /*Initialize the timer*/
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  // Use PWM mode, which sets output on overflow and clears on compare events
  timerInit.prescale = timerPrescale1;
  timerInit.enable = false;
  timerCCInit.mode = timerCCModePWM;
  timerCCInit.prsOutput = timerPrsOutputLevel;

  // configure, but do not start timer
  TIMER_Init(TIMER2, &timerInit);

  // Configure CC Channel 0
  TIMER_InitCC(TIMER2, 0, &timerCCInit);

  dutyCycle = DUTY_CYCLE_STEPS;
  // set PWM period
  timer2Freq = CMU_ClockFreqGet(cmuClock_TIMER2) / (timerInit.prescale + 1);
  topValue = (timer2Freq / CARRIER_FREQUENCY);

  // Set top value to overflow at the desired PWM_FREQ frequency
  TIMER_TopSet (TIMER2, topValue);
  /*Set compare value for initial duty cycle with the CCV register*/
  TIMER_CompareSet(TIMER2, 0, (uint32_t)(topValue * dutyCycle));

  TIMER_Enable(TIMER2, true);
}

/**************************************************************************//**
 * @brief
 *  IR generate time base
 *****************************************************************************/

void IR_generate_timebase(void)
{
  CMU_ClockEnable(cmuClock_TIMER1, true);
  count = 0;
  /*Initialize the timer*/
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

  timerInit.prescale = timerPrescale8;
  timerInit.enable = false;
  timerInit.clkSel = timerClkSelHFPerClk;

  // configure, but do not start timer
  TIMER_Init(TIMER1, &timerInit);

  timer1Freq = CMU_ClockFreqGet(cmuClock_TIMER1)/(timerInit.prescale + 1);
  emberAfCorePrintln("%timer1 frequency : %d", timer1Freq);
  ir_frame_setup();

  /* CC0 */
  TIMER_InitCC_TypeDef timerCC0Init = TIMER_INITCC_DEFAULT;
  timerCC0Init.mode = timerCCModeCompare;
  timerCC0Init.prsOutput = timerPrsOutputLevel;
  timerCC0Init.cofoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER1, 0, &timerCC0Init);

#ifdef IR_SEND_DEBUG
  GPIO->TIMERROUTE[1].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[1].CC0ROUTE = (SEND_PORT_BRD4182A << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                  | (0 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
#endif

  /* Load two first values in TIMER1 */
  ir_init_topB();

  /* Setup DMA */
  unsigned int channel;
  /* Initialize DMA */
  DMADRV_Init();
  /* Request a DMA channel */
  DMADRV_AllocateChannel( &channel, NULL );
  DMADRV_MemoryPeripheral(channel,
                          dmadrvPeripheralSignal_TIMER1_UFOF,
                          &(TIMER1->TOPB),
                          &IR_frame[2],
                          1,
                          length_IR_frame - 2,
                          dmadrvDataSize4,
                          NULL,
                          NULL);

  /* Clear and enable OF interrupt */
  TIMER_IntClear(TIMER1, TIMER_IF_OF);
  TIMER_IntEnable(TIMER1, TIMER_IF_OF);
  NVIC_EnableIRQ(TIMER1_IRQn);

  /* Start timer */
  TIMER_Enable(TIMER1, true);
}

void initPrs_And(void)
{
  /*init CMU*/
  CMU_ClockEnable(cmuClock_PRS, true);

  // Route PRS TIMER1 CC0 on PRS_CH + 1
  PRS_SourceAsyncSignalSet(
    PRS_CH + 1,
    PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER1,
    _PRS_ASYNC_CH_CTRL_SIGSEL_TIMER1CC0);

  // Route TIMER2 CC0 on PRS_CH
  PRS_SourceAsyncSignalSet(
    PRS_CH,
    PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER2,
    _PRS_ASYNC_CH_CTRL_SIGSEL_TIMER2CC0);

  // Configure PRS logic
  PRS_Combine(PRS_CH+1, PRS_CH, prsLogic_A_AND_B);

  // Route output
  PRS_PinOutput(PRS_CH+1, prsTypeAsync, SEND_PORT_BRD4182A , 1);
}


void IR_init_send()
{
  /*init CMU*/
  CMU_ClockEnable(cmuClock_GPIO, true);
  /*init Send PORT*/
  GPIO_PinModeSet(SEND_PORT_BRD4182A, SEND_PIN_BRD4182A, gpioModePushPull, 0);
  GPIO_PinModeSet(SEND_PORT_BRD4182A, 1, gpioModePushPull, 0);

  initPrs_And();

  //init 38kHZ
  IR_generate_freq();
  //send trame
  IR_generate_timebase();
}

#endif /* IR_SEND_C_ */
