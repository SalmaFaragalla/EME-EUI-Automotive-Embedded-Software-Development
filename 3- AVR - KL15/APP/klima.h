/*
 * klima.h
 *
 * Created: 8/26/2023 3:23:27 PM
 *  Author: Ahmed hisham
 */ 


#ifndef KLIMA_H_
#define KLIMA_H_

#include "LED.h"
#include "Timer.h"
#include "LCD.h"
#include "ADC.h"
#include "Ext_INT.h"

typedef enum
{
	OFF,
	READY,
	IGNITION
	}Klima_States;

void Klima_init(void);
void Klima_runnable(void);


#endif /* KLIMA_H_ */