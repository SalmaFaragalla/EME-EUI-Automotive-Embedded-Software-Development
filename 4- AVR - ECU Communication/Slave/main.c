/*
 * Sunrise_alarm.c
 *
 * Created: 28-Aug-23 3:08:47 PM
 * Author : Salma
 */ 



#include "Vehicle_Project_Slave.h"
#include "STD_TYPES.h"
#include "Keypad.h"
extern u16 Adc_Data;
extern u8 key;
int main(void)
{
    Car_SlaveInit();
               
    while (1)
    {
     
           Car_SlaveRunnable();
            
    }

 

}
