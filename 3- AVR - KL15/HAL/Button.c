/*
 * Button.c
 *
 * Created: 8/20/2023 11:08:45 AM
 *  Author: nora
 */ 

#include "Button.h"
#include "DIO.h"

extern TRAFFIC_MODE mode;
extern long long j ;

void Button_Init(uint8_t Port , uint8_t Pin){
	DIO_SetPinDir(Port, Pin,DIO_PIN_INPUT);
	DIO_SetPullUp(Port,Pin);
}

void Button_Pressed(void){
    if(mode!=Pedestrian)
    j=0;
    mode=Pedestrian;
    
}

