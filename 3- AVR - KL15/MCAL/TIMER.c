/*
 * TIMER.c
 *
 * Created: 8/25/2023 11:38:10 AM
 *  Author: Norhan Mohamed
 */ 

#include "TIMER.h"
#include "Bit_math.h"
#include "STD_types.h"
#include <avr/interrupt.h>


/*******************************************************************************
 *                           Global Variables                                  *
 *******************************************************************************/

/* Global variables to hold the address of the call back function in the application */
static volatile void (*g_timer0CallBackPtr)(void)=NULL_PTR;

ErrorType Error_Init;
ErrorType Error_Duty;



bool TIMER_Init(const TIMER_ConfigType * Config_ptr){
	ClEAR_BIT(TCCR0,FOC0);
	if (Config_ptr-> mode == NORMAL_MODE || 
	   Config_ptr-> mode == COMPARE_MODE || 
	   Config_ptr-> mode == FAST_PWM     ||
	   Config_ptr-> mode == PHASE_CONTROL )
	   { 
		TCCR0 = (TCCR0 & 0xB7 ) | (Config_ptr-> mode <<3);
		
		if(Config_ptr->prescaler < 8 &&  Config_ptr->prescaler > -1 )
		{
		 TCCR0 = (TCCR0 & 0xF8 ) | (Config_ptr->prescaler);
	
			if (Config_ptr-> mode == NORMAL_MODE)
			{
				if(Config_ptr->StartValue <= 255 &&  Config_ptr->StartValue > -1)
				{
					TCNT0 = Config_ptr->StartValue;
					SET_BIT(TIMSK,TOIE0); 
				}
				else 
					Error_Init = ErrorInStartValue;
			}
			else if (Config_ptr-> mode == COMPARE_MODE)
			{
				if (Config_ptr->CompareOutput < 4 &&  Config_ptr->CompareOutput > -1)
				{
					if(Config_ptr->CompareValue <= 255 &&  Config_ptr->CompareValue > -1)
					{
						TCCR0 = (TCCR0 & 0xCF ) | (Config_ptr-> CompareOutput <<4);
						TCNT0  =0;
						OCR0 = Config_ptr ->CompareValue;
						SET_BIT(TIMSK,OCIE0);
					}
					else 
						Error_Init = ErrorInCompareValue;
				}
				
				else
					Error_Init = ErrorInCompareMode;
			}
			else if (Config_ptr-> mode == FAST_PWM)
			{
				if(Config_ptr->PwmOutput < 4)
				{
					TCCR0 = (TCCR0 & 0xCF ) | (Config_ptr-> PwmOutput <<4);
					TCNT0  =0;
					OCR0=0;
					if(Config_ptr->StartValue <= 255 &&  Config_ptr->StartValue > -1)
					{
						TCNT0 = Config_ptr->StartValue;
						SET_BIT(TIMSK,TOIE0);
					}
				}
				else 
					Error_Init = ErrorInPwmMode;
				
			}
		}
			else 
			Error_Init = ErrorInPrescaler;
	   }
	   else 
	   Error_Init = ErrorInMode;
	   
	   
	 if (Error_Init == 0)
	 {
		 return SUCCESSFUL;
	 }  
	 else 
		return UNSUCCESSFUL;
}


bool TIMER_SetDuty(uint8_t duty){
	if(duty <= 100 && duty > -1)
	{
		OCR0= (duty*255)/100;
	}
	
	else 
		Error_Duty = ErrorInDuty;
	
	if (Error_Duty == 0)
	 {
		 return SUCCESSFUL;
	 }
	else
		 return UNSUCCESSFUL;
	
}

void TIMER0_setCallBack(void (*a_ptr)(void)){
	
	g_timer0CallBackPtr=a_ptr;
}

ISR(TIMER0_OVF_vect){
	if(g_timer0CallBackPtr != NULL_PTR){
		g_timer0CallBackPtr();
	}
}
