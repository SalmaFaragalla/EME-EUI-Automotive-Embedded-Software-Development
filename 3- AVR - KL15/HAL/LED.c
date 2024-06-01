/*
 * LED.c
 *
 * Created: 20-Aug-23 10:37:42 AM
 *  Author: Salma
 */

#include "LED.h" 
#include "DIO.h"
#define F_CPU 16000000UL
#include <util/delay.h>




void LED_Init(void)
{

        DIO_SetPinDir(LED_GREEN_PORT,LED_GREEN_PIN,DIO_PIN_OUTPUT);
        DIO_SetPinDir(LED_RED_PORT,LED_RED_PIN,DIO_PIN_OUTPUT);
        DIO_SetPinDir(LED_BLUE_PORT,LED_BLUE_PIN,DIO_PIN_OUTPUT);
        DIO_SetPinDir(LED_YELLOW_PORT,LED_YELLOW_PIN,DIO_PIN_OUTPUT);  
    
}
void LED_deinit(uint8_t led_color)
{
	switch (led_color)
	{
		case LED_GREEN :
		DIO_SetPinDir(LED_GREEN_PORT,LED_GREEN_PIN,DIO_PIN_INPUT);
		break;
		
		case LED_RED :
		DIO_SetPinDir(LED_RED_PORT,LED_RED_PIN,DIO_PIN_INPUT);
		break;
		
		case LED_BLUE :
		DIO_SetPinDir(LED_BLUE_PORT,LED_BLUE_PIN,DIO_PIN_INPUT);
		break;
		
		case LED_YELLOW :
		DIO_SetPinDir(LED_YELLOW_PORT,LED_YELLOW_PIN,DIO_PIN_INPUT);
		break;
		
	}
}
void LED_TurnOn(uint8_t led_color)
{
    switch (led_color)
    {
        case LED_GREEN : 
            DIO_SetPinVal(LED_GREEN_PORT,LED_GREEN_PIN,DIO_PIN_HIGH);              
            break;
       
       case LED_RED :
            DIO_SetPinVal(LED_RED_PORT,LED_RED_PIN,DIO_PIN_HIGH);
            break;   
            
       case LED_BLUE :
           DIO_SetPinVal(LED_BLUE_PORT,LED_BLUE_PIN,DIO_PIN_HIGH);
           break; 
           
       case LED_YELLOW :
           DIO_SetPinVal(LED_YELLOW_PORT,LED_YELLOW_PIN,DIO_PIN_HIGH);
           break;                       
        
    }
    
}

void LED_TurnOff(uint8_t led_color)
{
    switch (led_color)
    {
        case LED_GREEN :
            DIO_SetPinVal(LED_GREEN_PORT,LED_GREEN_PIN,DIO_PIN_LOW);
            break;
        
        case LED_RED :
            DIO_SetPinVal(LED_RED_PORT,LED_RED_PIN,DIO_PIN_LOW);
            break;
        
        case LED_BLUE :
            DIO_SetPinVal(LED_BLUE_PORT,LED_BLUE_PIN,DIO_PIN_LOW);
            break;
        
        case LED_YELLOW :
            DIO_SetPinVal(LED_YELLOW_PORT,LED_YELLOW_PIN,DIO_PIN_LOW);
            break;
        
    }
    
}

/*Blinks for 5 Sec*/
void LED_Blink_5sec(uint8_t led_color)
{
    int LED_Counter=0;
        switch (led_color)
        {
            case LED_GREEN :
                DIO_SetPinDir(LED_GREEN_PORT,LED_GREEN_PIN,DIO_PIN_OUTPUT);
                
                for (LED_Counter = 0 ; LED_Counter<LED_5sec_Count ; ++LED_Counter)
                {
                    DIO_SetPinVal(LED_GREEN_PORT,LED_GREEN_PIN,DIO_PIN_HIGH);
                    _delay_ms(LED_BLINK_TIME);
                    DIO_SetPinVal(LED_GREEN_PORT,LED_GREEN_PIN,DIO_PIN_LOW);
                    _delay_ms(LED_BLINK_TIME);
                }
                break;
            
            case LED_RED :
                DIO_SetPinDir(LED_RED_PORT,LED_RED_PIN,DIO_PIN_OUTPUT);
                
                for (LED_Counter = 0 ; LED_Counter<LED_5sec_Count ; ++LED_Counter)
                {
                    DIO_SetPinVal(LED_RED_PORT,LED_RED_PIN,DIO_PIN_HIGH);
                    _delay_ms(LED_BLINK_TIME);
                    DIO_SetPinVal(LED_RED_PORT,LED_RED_PIN,DIO_PIN_LOW);
                    _delay_ms(LED_BLINK_TIME);
                }
                break;
            
            case LED_BLUE :
                DIO_SetPinDir(LED_BLUE_PORT,LED_BLUE_PIN,DIO_PIN_OUTPUT);
                for (LED_Counter = 0 ; LED_Counter<LED_5sec_Count ; ++LED_Counter)
                {
                    DIO_SetPinVal(LED_BLUE_PORT,LED_BLUE_PIN,DIO_PIN_HIGH);
                    _delay_ms(LED_BLINK_TIME);
                    DIO_SetPinVal(LED_BLUE_PORT,LED_BLUE_PIN,DIO_PIN_LOW);
                    _delay_ms(LED_BLINK_TIME);
                }
                break;

            
            case LED_YELLOW :
                DIO_SetPinDir(LED_YELLOW_PORT,LED_YELLOW_PIN,DIO_PIN_OUTPUT);
                
                for (LED_Counter = 0 ; LED_Counter<LED_5sec_Count ; ++LED_Counter)
                {
                    DIO_SetPinVal(LED_YELLOW_PORT,LED_YELLOW_PIN,DIO_PIN_HIGH);
                    _delay_ms(LED_BLINK_TIME);
                    DIO_SetPinVal(LED_YELLOW_PORT,LED_YELLOW_PIN,DIO_PIN_LOW);
                    _delay_ms(LED_BLINK_TIME);
                }
                break;            
            
        }
        
  
}

