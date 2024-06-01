/*
 * DIO.c
 *
 * Created: 17-Aug-23 2:29:20 PM
 * Author : Salma
 */ 

#include "LCD.h"
#include "Ext_INT.h"
#include "LED.h"
#include "Button.h"
#include "Traffic_Light.h"

#include <stdio.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#include <avr/interrupt.h>

int main(void)
{
        
    traffic_light_Init();

    while (1)
    {
        traffic_light_Runnable();
   

    }

}