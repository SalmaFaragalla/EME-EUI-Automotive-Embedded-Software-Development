/*
 * Sunrise_alarm.c
 *
 * Created: 28-Aug-23 3:08:47 PM
 * Author : Salma
 */ 



#include "Vehicle_comm.h"
#include "STD_TYPES.h"
#include "Keypad.h"
extern u16 Adc_Data;
extern u8 key;
int main(void)
{
	keyPad_Init();
    APP_Init();
	
               
    while (1)
    {
     
            APP_runnable();
			
            
    }

 

}
