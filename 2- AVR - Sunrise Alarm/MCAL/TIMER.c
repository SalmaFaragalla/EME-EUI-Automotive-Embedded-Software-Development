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

static volatile void (*g_timer2CallBackPtr)(void)=NULL_PTR;

ErrorType Error_Init2;
ErrorType Error_Duty2;
static int OVF_Counter = 0;
	 
bool TIMER2_Init(const TIMER_ConfigType * Config_ptr){
	CLR_BIT(TCCR2,FOC2);
	if (Config_ptr-> mode == NORMAL_MODE ||
	Config_ptr-> mode == COMPARE_MODE ||
	Config_ptr-> mode == FAST_PWM     ||
	Config_ptr-> mode == PHASE_CONTROL )
	{
		TCCR2 = (TCCR2 & 0xB7 ) | (Config_ptr-> mode <<3);
		
		if(Config_ptr->prescaler < 8 &&  Config_ptr->prescaler > -1 )
		{
			TCCR2 = (TCCR2 & 0xF8 ) | (Config_ptr->prescaler);
			
			if (Config_ptr-> mode == NORMAL_MODE)
			{
				if(Config_ptr->StartValue <= 255 &&  Config_ptr->StartValue > -1)
				{
					TCNT2 = Config_ptr->StartValue;
					SET_BIT(TIMSK,TOIE2);
				}
				else
				Error_Init2 = ErrorInStartValue;
			}
			else if (Config_ptr-> mode == COMPARE_MODE)
			{
				if (Config_ptr->CompareOutput < 4 &&  Config_ptr->CompareOutput > -1)
				{
					if(Config_ptr->CompareValue <= 255 &&  Config_ptr->CompareValue > -1)
					{
						TCCR2 = (TCCR2 & 0xCF ) | (Config_ptr-> CompareOutput <<4);
						TCNT2  =0;
						OCR2 = Config_ptr ->CompareValue;
						SET_BIT(TIMSK,OCIE2);
					}
					else
					Error_Init2 = ErrorInCompareValue;
				}
				else
				Error_Init2 = ErrorInCompareMode;
			}
			else if (Config_ptr-> mode == FAST_PWM)
			{
				if(Config_ptr->PwmOutput < 4)
				{
					TCCR2 = (TCCR2 & 0xCF ) | (Config_ptr-> PwmOutput <<4);
					TCNT2  =0;
					OCR2=0;
					if(Config_ptr->StartValue <= 255 &&  Config_ptr->StartValue > -1)
					{
						TCNT2 = Config_ptr->StartValue;
						SET_BIT(TIMSK,TOIE2);
					}
					
				}
				else
				Error_Init2 = ErrorInPwmMode;
				
			}
		}
		else
		Error_Init2 = ErrorInPrescaler;
	}
	else
	Error_Init2 = ErrorInMode;
	
	
	if (Error_Init2 == 0)
	{
		return SUCCESSFUL;
	}
	else
	return UNSUCCESSFUL;

}
	


enu_timer0_error_status_t TIMER0_Init(st_timer0_config_t_ *st_arg_timer0_config)
{
	enu_timer0_error_status_t ret_timer0_status = ERROR_TIMER0_OK;

	if (st_arg_timer0_config->enu_timer0_prescaler > TIMER0_PRESCALER_TOTAL || st_arg_timer0_config->enu_timer0_mode > TIMER0_MODES_TOTAL || st_arg_timer0_config->enu_timer0_int_en > TIMER0_INT_EN_TOTAL || st_arg_timer0_config->enu_timer0_oc0 > TIMER0_OC0_TOTAL || st_arg_timer0_config->preload > TIMER_MAX_REG_VALUE || st_arg_timer0_config->compare_val > TIMER_MAX_REG_VALUE)
	{
		ret_timer0_status = ERROR_TIMER0_NOK;
		return ret_timer0_status;
	}
	/* Set mode */
	switch (st_arg_timer0_config->enu_timer0_mode)
	{
		case CTC:
		SET_BIT(TCCR0, WGM01);
		CLR_BIT(TCCR0, WGM00);
		SET_BIT(TCCR0, 7);
		break;

		case NORMAL:
		CLR_BIT(TCCR0, WGM00);
		CLR_BIT(TCCR0, WGM01);
		SET_BIT(TCCR0, 7);
		break;

		case PWM_FAST:
		SET_BIT(TCCR0, 3);
		SET_BIT(TCCR0, 6);
		CLR_BIT(TCCR0, 7);
		break;

		case PWM_PHASE_CORRECT:
		CLR_BIT(TCCR0, 3);
		SET_BIT(TCCR0, 6);
		CLR_BIT(TCCR0, 7);
		break;
		default:
		ret_timer0_status = ERROR_TIMER0_NOK;
		break;
	}

	/* Set Preload */
	OCR0 = st_arg_timer0_config->compare_val;
	/* Set Prescaler */
	TCCR0 |= st_arg_timer0_config->enu_timer0_prescaler;
	/* Set OCR */
	TCCR0 |= ((st_arg_timer0_config->enu_timer0_oc0) << 4);
	/* Set Interrupt */
	if ((st_arg_timer0_config->enu_timer0_int_en) != POLLING)
	{
		sei();
		TIMSK |= st_arg_timer0_config->enu_timer0_int_en;
	}
	/* Set Preload value */
	TCNT0 |= st_arg_timer0_config->preload;

	return ret_timer0_status;
}

enu_timer0_error_status_t TIMER0_set_compare(u8 u8_arg_compare_value)
{
	enu_timer0_error_status_t ret_timer0_status = ERROR_TIMER0_OK;

	if (u8_arg_compare_value > TIMER_MAX_REG_VALUE)
	{
		ret_timer0_status = ERROR_TIMER0_NOK;
		return ret_timer0_status;
	}
	OCR0 = u8_arg_compare_value;
	return ret_timer0_status;
}


void TIMER0_Deinit(void)
{
	TCCR0 = 0x00;
	TIMSK = 0x00;
	TCNT0 = 0x00;
}
ISR(TIMER0_OVF_vect)
{
	++OVF_Counter;
	if (OVF_Counter == 31)
	{
		TOG_BIT(PORTA, 5);
		OVF_Counter = 0;
	}
}
	

bool TIMER2_SetDuty(uint8_t duty){
	if(duty <= 100 && duty > -1)
	{
		OCR2= (float)(duty*255)/100;
	}
	
	else
	Error_Duty2 = ErrorInDuty;
	
	if (Error_Duty2 == 0)
	{
		return SUCCESSFUL;
	}
	else
	return UNSUCCESSFUL;
	
}



void TIMER2_setCallBack(void (*a_ptr)(void)){
	
	g_timer2CallBackPtr=a_ptr;
}

ISR(TIMER2_OVF_vect){
	if(g_timer2CallBackPtr != NULL_PTR){
		(*g_timer2CallBackPtr)();
	}
}
