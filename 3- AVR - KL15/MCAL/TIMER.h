/*
 * TIMER.h
 *
 * Created: 8/25/2023 11:38:25 AM
 *  Author: Norhan Mohamed
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include "STD_types.h"
#include "DIO.h"
#include <avr/interrupt.h>

#define FOC0   DIO_PIN7
#define TOIE0  DIO_PIN0
#define OCIE0  DIO_PIN1

/*******************************************************************************
* Types Declaration                                   *
 *******************************************************************************/

typedef enum{
	NORMAL_MODE,COMPARE_MODE=1, FAST_PWM=9,PHASE_CONTROL=8
}TIMER_modeType;

typedef enum{
	NO_CLOCK,F_CPU_CLOCK,F_CPU_8,F_CPU_64,F_CPU_256,F_CPU_1024,EXT_CLOCK_FALLING_EDGE,EXT_CLOCK_RISING_EDGE
}TIMER_prescaler;

typedef enum{
	NORMAL,TOGGLE,CLEAR,SET
}TIMER_CompareModeType;

typedef enum{
	NO_MODE,RESERVED,NON_INVERTING,INVERTING
}TIMER_PwmModeType;

typedef enum{
	ErrorInMode, 
	ErrorInPrescaler,
	ErrorInCompareMode,
	ErrorInPwmMode,
	ErrorInCompareValue,
	ErrorInStartValue,
	ErrorInDuty
	}ErrorType;
	
typedef enum {
	SUCCESSFUL,
	UNSUCCESSFUL
	}bool;



typedef struct {
	TIMER_modeType mode;
	TIMER_prescaler prescaler;
	uint8_t StartValue;
	uint8_t CompareValue;
	TIMER_CompareModeType CompareOutput; 
	TIMER_PwmModeType PwmOutput;
	
	}TIMER_ConfigType;
	
bool TIMER_Init(const TIMER_ConfigType * Config_ptr);
bool TIMER_SetDuty(uint8_t duty);
void TIMER0_setCallBack(void (*a_ptr)(void));




#endif /* TIMER_H_ */