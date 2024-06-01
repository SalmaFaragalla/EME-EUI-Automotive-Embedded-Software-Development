/*
 * klima.c
 *
 * Created: 8/26/2023 3:23:16 PM
 *  Author: Ahmed hisham
 */ 

#include "klima.h"
#include <avr/delay.h>
uint8_t Flag_Ready=0;
Klima_States state;
volatile uint8_t Flag_Dim;
extern uint16_t Adc_Data;
uint8_t Timer_count=0;
uint8_t klima_seconds=0;
Klima_States state;

void klima_Timer(void){
 
   Timer_count++;
 if(Timer_count==62){
	 if(state==READY &&Flag_Dim==0){
	 klima_seconds++;
	 }
	 Timer_count=0;
 }
 if( klima_seconds==10){
	 Flag_Dim=1;
	 klima_seconds=0;
 }
	
	
}
void Klima_init(void)
{
	LCD_Init();
	LED_Init();
	SET_GlobalInterrupt();
	TIMER_ConfigType ConfigStruct = {FAST_PWM,F_CPU_1024,0,0,NORMAL,NON_INVERTING};
	TIMER_Init(&ConfigStruct);
	TIMER0_setCallBack(klima_Timer);
	ADC_Init( VREF_VCC, ADC_SCALER_128);
	DIO_SetPinDir(DIO_PORTB,DIO_PIN3,DIO_PIN_OUTPUT);	
	LED_deinit(LED_BLUE);
	

}
void Klima_runnable(void){
	
	ADC_ReadChannel(CHANNEL0);
	if(Adc_Data<10){
		state=OFF;
		
	}
	else if(Adc_Data<28){
		state=READY;
	}
	else{
		state=IGNITION;
	}
	
	_delay_ms(100);

	if(state==OFF){
	LED_TurnOff(LED_GREEN);
	LCD_Clear();
	LCD_WriteString("OFF");
	Flag_Ready=0;
	Flag_Dim=0;
	TIMER_SetDuty(0);
	}
  else if(state==READY){
	  if(Flag_Ready==0){
		TCNT0=246;
		Flag_Ready=1;
		
	  }
	  
	 if(Flag_Dim==1){
		 LED_TurnOff(LED_GREEN);
		 TIMER_SetDuty(10);
		
	 }
	 else{
		 	
		 LED_TurnOn(LED_GREEN);
		 
	 }
	 LCD_Clear();
	  LCD_WriteString("READY");
	  LCD_WriteNumber(klima_seconds);
  }
	else{
		LCD_Clear();
		 LCD_WriteString("IGNITION");
		 LED_TurnOff(LED_GREEN);
		 TIMER_SetDuty(0);
		 Flag_Ready=0;
		 Flag_Dim=0;
	}
	
	
}