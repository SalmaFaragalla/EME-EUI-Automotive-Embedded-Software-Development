/*
 * sunrise_alarm.h
 *
 * Created: 8/28/2023 3:05:43 PM
 *  Author:nada
 */ 


#ifndef SUNRISE_ALARM_H_
#define SUNRISE_ALARM_H_
/*******************************************************************************
 *                                  Includes                                   *
 *******************************************************************************/
#include "LCD.h"
#include "Timer.h"
#include "ADC.h"
#include "LED_interface.h"
#include "Keypad.h"
#include "Buzzer_interface.h"
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>

/*******************************************************************************
 *                                  Type Declaration                           *
 *******************************************************************************/
typedef enum {
	ERROR,
	NO_ERROR,
	Time_deleted_successfully,
	Timer_Not_found
	}error_status;

typedef enum {
	MATCHED=1,
	NO_MATCH=5
}match_status;


typedef struct
{
	signed char seconds_user;
	signed char minutes_user;
	signed char minutes_relative;
	signed char seconds_relative;
	}alarm;
	
/*******************************************************************************
 *                             Function Prototypes     		                   *
 *******************************************************************************/	
	
void sorting_timers(alarm* INPUT_TIMERS);
error_status SET(void);
error_status cancel_timer(uint8_t Timer_num);	
error_status Waking_sequence_Init(void);
void Display_menu(void);
void Init(void);
void runnable(void);
error_status Timers_Display(void);
void count_time();
void MATCH(void);
u8 LDR_check(void);
#endif /* SUNRISE_ALARM_H_ */