/*
 * simple_RTOS.h
 *
 *  Created on: Oct 13, 2023
 *      Author: Salma
 */

#ifndef INC_SIMPLE_RTOS_H_
#define INC_SIMPLE_RTOS_H_

#include "stm32f1xx_hal.h"

# define NUM_OF_TASKS 3

/* Tasks Periodicity*/
# define LED_RED_PERIOD 100
# define LED_YELLOW_PERIOD 200
# define LED_GREEN_PERIOD 500

/* Tasks Priorities*/
# define LED_RED_PRIORITY    0
# define LED_YELLOW_PRIORITY 1
# define LED_GREEN_PRIORITY  2

/* Tasks Arguments*/
# define LED_RED_ARG GPIOA,GPIO_PIN_1
# define LED_YELLOW_ARG GPIOC,GPIO_PIN_13
# define LED_GREEN_ARG GPIOC,GPIO_PIN_15

/* Tasks Type definition*/
typedef struct
{
	uint16_t periodicity;
	void (*Fptr) (void);
}task_t;


/* Tasks  Functions*/
void BlinkRed (void);
void BlinkYellow (void);
void BlinkGreen (void);

/* RTOS functions*/
void CreateTask (uint8_t priority , uint16_t periodicity , void (*Fptr) (void));
void Scheduler (void);


#endif /* INC_SIMPLE_RTOS_H_ */
