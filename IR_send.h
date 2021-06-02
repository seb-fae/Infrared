/*
 * IR_send.h
 *
 *  Created on: Mar 22, 2021
 *      Author: abl
 */

#ifndef IR_SEND_H_
#define IR_SEND_H_
#include "em_gpio.h"
#include "em_timer.h"


#endif /* IR_SEND_H_ */

/* DEFINITION */

/*EXP HEADER 9 on BRD4183A*/
#define SEND_PORT_BRD4183A gpioPortB
#define SEND_PIN_BRD4183A 1
/*EXP HEADER 15 on BRD4182A*/
#define SEND_PORT_BRD4182A gpioPortB
#define SEND_PIN_BRD4182A 0


//CHANGE TO TYPE DEF STRUCT
#define PWM_FREQ 38000
#define DUTY_CYCLE_STEPS  0.335
#define HALF_BIT_FREQ 1777

#define STREAM_BIT_NUM 200 // à augmenter à 300*4 dans le cas d'une clim
#define TABLE_INDEX_NUM					18
#define NEC_REPEAT_HEAD_SPACE_BIT_SIZE	4
//#define BIT(n)							(1<<n)


typedef enum {
  CODE_NEC,
  CODE_CLIM,
  CODE_NUM
}code_t;

typedef enum {
  HEAD_PULSE,
  HEAD_SPACE,
  HEAD_NUM
}head_t;

typedef struct {
  code_t code;
  uint16_t carrier[CODE_NUM];
  uint16_t timebase[CODE_NUM];
  float dutycycle[CODE_NUM];
  uint8_t head_bit_size[CODE_NUM][HEAD_NUM];
  //uint8_t address_length[CODE_NUM];
  //uint8_t command_length[CODE_NUM];
  uint8_t stream_length[CODE_NUM];

  uint8_t index;
  uint8_t stream_index;
  bool stream_active;
  uint8_t stream[STREAM_BIT_NUM];
}ir_t;

/*Utiliser plus tard un call_back qui permet de savoir si la transmission est finie ou non*/
typedef void (*ir_callback_t)(void);


/* FUNCTION */

void IR_init_send(void);

void IR_generate_begin(void);

void IR_generate_stream(void);

void IR_generate_freq(void);

void IR_generate_timebase(void);




