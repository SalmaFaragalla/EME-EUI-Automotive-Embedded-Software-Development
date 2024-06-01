/*
 * Ext_INT.h
 *
 * Created: 8/19/2023 10:11:44 AM
 *  Author: eme
 */ 


#ifndef EXT_INT_H_
#define EXT_INT_H_

#include "Ext_INT_Cfg.h"
#include <avr/interrupt.h>

#define GLOBAL_INTERRUPT_ENABLE	 0
#define GLOBAL_INTERRUPT_DISABLE 1


/*GICR – General Interrupt Control Register ,These are the bit numbers */

#define INterrupt_ON EXTENAL_INT1


#define EXTENAL_INT0 6
#define EXTENAL_INT1 7
#define EXTENAL_INT2 5


#define INT0_TRIGGER_LOW_LEVEL			0
#define INT0_TRIGGER_RISING_EDGE		1
#define INT0_TRIGGER_FALLING_EDGE		2
#define INT0_TRIGGER_ANY_LOGICAL_CHANGE 3

#define INT1_TRIGGER_LOW_LEVEL			0
#define INT1_TRIGGER_RISING_EDGE		1
#define INT1_TRIGGER_FALLING_EDGE		2
#define INT1_TRIGGER_ANY_LOGICAL_CHANGE 3

#define INT2_TRIGGER_RISING_EDGE		0
#define INT2_TRIGGER_FALLING_EDGE		1

void SET_GlobalInterrupt (void);

void EXTERNAL_INT0_Init (void);
void EXTERNAL_INT1_Init (void);
void EXTERNAL_INT2_Init (void);
void EXTERNAL_INT0_Disable (void);
void EXTERNAL_INT1_Disable (void);
void EXTERNAL_INT2_Disable (void);

void EXI_SetCallBack(void(*LocalPtr)(void));
#endif /* EXT_INT_H_ */