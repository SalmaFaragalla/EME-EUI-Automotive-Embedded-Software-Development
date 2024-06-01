/*
 * Traffic_light.c
 *
 * Created: 8/20/2023 10:52:55 AM
 *  Author: eme
 */

#include "LCD.h"
#include "Ext_INT.h"
#include "LED.h"
#include "Button.h"
#include "DIO.h"
#include "Traffic_Light.h"
#define F_CPU 16000000UL
#include <util/delay.h>

volatile TRAFFIC_MODE mode = CAR;
volatile long long  i;
volatile long long j =1000000;
void traffic_light_Init(void)
{

	SET_GlobalInterrupt();

	EXTERNAL_INT1_Init();
	EXI_SetCallBack(Button_Pressed);
	LCD_Init();
	LED_Init();
	Button_Init(DIO_PORTD, DIO_PIN3);
}
void traffic_light_Runnable(void)
{
    LCD_Clear();
	LCD_WriteString("Cars");
	LED_TurnOn(LED_GREEN);
	
	Delay_5sec();

	LED_TurnOff(LED_GREEN);

	LED_Blink_5sec(LED_YELLOW);
    j=1000000;
	LED_TurnOn(LED_RED);
	if (mode == Pedestrian)
	{
		LCD_Clear();
        LCD_Clear();
		LCD_WriteString("Pedestrian");
		LED_TurnOn(LED_BLUE);
		Delay_5sec();
		
        	
		mode = CAR;
		LED_TurnOff(LED_RED);
		LED_TurnOff(LED_BLUE);
	}
	else
	{
		Delay_5sec();
        
     
		LED_TurnOff(LED_RED);
        	
		if (mode == Pedestrian)
		{
			LCD_Clear();
			LCD_WriteString("Pedestrian");
			LED_TurnOn(LED_RED);
			LED_TurnOn(LED_BLUE);
			_delay_ms(5000);

			LED_TurnOff(LED_RED);
			LED_TurnOff(LED_BLUE);
			mode = CAR;
		}
		else
		{
			// Do_Nothing
		}
	}
}
	
	void Delay_5sec(void)
	{
		for(i=0;i<j;i++);
        j=1000000;

	}
	