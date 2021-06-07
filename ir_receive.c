#ifndef IR_SEND_C_
#define IR_SEND_C_

#include "ir.h"
#include "app/framework/include/af.h"

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "dmadrv.h"

uint32_t receive_len;
uint32_t *receive_data;
unsigned int dma_channel;
uint32_t timerFreq = 0;

void ir_frame_setup()
{
   for (uint32_t i = 0; i < receive_len; i++)
   {
      /* Convert sample to us */
      uint64_t v = ((uint64_t)receive_data[i] * (uint64_t)1000000)/timerFreq;
  #ifdef IR_RECEIVE_DEBUG
      emberAfCorePrintln("v[%d] = %ld", i, v);
  #endif
      receive_data[i] = (uint32_t)v;
   }
}

void end_of_transfer()
{
  /* Stop Timer */
  TIMER_Enable(TIMER1,false);
  /* Convert sample to us */
  ir_frame_setup();
  /* Application callback */
  ir_receive_callback(receive_len - remaining);
}

void ir_init(void)
{
	/* Init CMU */
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	/* Configure PB0 as input */
	GPIO_PinModeSet(gpioPortB, 0, gpioModeInput, 0);
	/* Init Send PORT */
	GPIO_PinModeSet(SEND_PORT_BRD4182A, SEND_PIN_BRD4182A, gpioModePushPull, 0);
}

/* This is called when input buffer if full */
DMADRV_Callback_t dma_done(void)
{
  end_of_transfer();
  return 0;
}

void TIMER1_IRQHandler(void)
{
  /* Acknowledge the interrupt */
  uint32_t flags = TIMER_IntGet(TIMER1);
  TIMER_IntClear(TIMER1, flags);

  if (flags & TIMER_IF_OF)
  /* End of reception */
  {
    int remaining;
    /* Get how many symbols have been captured */
    DMADRV_TransferRemainingCount(dma_channel, &remaining);
    /* Terminate transfer */
    end_of_transfer();
  }
}

/**************************************************************************//**
 * @brief
 *    TIMER initialization
 *****************************************************************************/

void initTimer_Receive(void)
{
  // Initialize the timer
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

  timerInit.prescale = timerPrescale64;
  timerInit.enable = false;
  /*Simplifiy measured period calculation*/
  timerInit.fallAction = timerInputActionReloadStart;
  timerInit.riseAction = timerInputActionReloadStart;
  timerCCInit.edge = timerEdgeBoth;
  timerCCInit.mode = timerCCModeCapture;
  timerCCInit.filter = true;

  TIMER_Init(TIMER1, &timerInit);

  // Route TIMER1 CC0 output to PB0 (EXP HEADER 7)
  GPIO->TIMERROUTE[1].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[1].CC0ROUTE = (gpioPortB << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                    | (0 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  TIMER_InitCC(TIMER1, 0, &timerCCInit);

  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER1)/(timerInit.prescale + 1);

  TIMER_IntClear(TIMER1, TIMER_IF_CC0 | TIMER_IF_OF);
  NVIC_ClearPendingIRQ(TIMER1_IRQn);
  // Enable TIMER1 interrupts
  TIMER_IntEnable(TIMER1, TIMER_IEN_CC0 | TIMER_IEN_OF);
  NVIC_EnableIRQ(TIMER1_IRQn);

  /* Initialize DMA */
  DMADRV_Init();
  /* Request a DMA dma_channel */
  DMADRV_AllocateChannel( &dma_channel, NULL );
  DMADRV_PeripheralMemory(dma_channel,
                          dmadrvPeripheralSignal_TIMER1_CC0,
                          receive_data,
                          &(TIMER1->CC[0].ICF),
                          1,
                          receive_len,
                          dmadrvDataSize4,
                          &dma_done,
                          NULL);
}


void start_ir_receive(uint32_t *p, size_t len)
{
  receive_data = p;
  receive_len = len;
  initTimer_Receive();
}

#endif /* IR_SEND_C_ */
