/*
 * Vehicle_Project_Slave.c
 *
 * Created: 9/5/2023 4:37:18 PM
 *  Author: nada
 */ 
#include "SPI_Interface.h"
#include "BIT_MATH.h"
#include "ATMega32A_Registers.h"
#include "TWI_interface.h"
#include "EEPROM_Interface.h"
#include "LED_interface.h"
#include <avr/delay.h>
volatile u8 Speed_Limit=1;
volatile static u8 Flag=2; //0 for read and 1 for write
void slave_fun(void){
	
	if (SPI_Tranceive_NoBlock(&Speed_Limit)==Recive_NoBlock)
	{
		Flag=0;
		LED_Toggle(LED_YELLOW);
	}
	else{
		Flag=1;
		LED_Toggle(LED_BLUE);
	}
}


void Car_SlaveInit(void){
	SET_BIT(SREG,7); //Set global variable (Replace it with the function)
	SPI_SlaveInit();
    TWI_SETBit_Rate();
	TWI_voidInitMaster(0);
	SPI_SetCallBack(slave_fun);
	
	LED_Init(LED_YELLOW);
	LED_Init(LED_BLUE);
}


void Car_SlaveRunnable(void){
	if(Flag==0){
		EEPROM_voidSendDataByte(5,Speed_Limit);
		Speed_Limit=EEPROM_u8ReadDataByte(5);
		Flag=2;
		SPI_Transceive(Speed_Limit);
	}
	else if(Flag==1)
	{
		
	}
	
	
}
