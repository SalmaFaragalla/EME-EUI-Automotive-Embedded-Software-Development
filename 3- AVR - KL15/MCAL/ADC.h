/*
 * ADC.h
 *
 * Created: 8/24/2023 12:04:11 PM
 *  Author: eme
 */ 


#ifndef ADC_H_
#define ADC_H_

#include "STD_types.h"
#include <avr/interrupt.h>
#include "ATMega32A_Registers.h"
#include "Bit_math.h"

#define Mode   interrupt55

#define interrupt55 1
#define polling   2


typedef enum{
	VREF_AREF,
	VREF_VCC,
	VREF_256
	
}ADC_VoltRef_type;

typedef enum{
	ADC_SCALER_2=1,
	ADC_SCALER_4,
	ADC_SCALER_8,
	ADC_SCALER_16,
	ADC_SCALER_32,
	ADC_SCALER_64,
	ADC_SCALER_128,
}ADC_Prescaler_type;


typedef enum {
	CHANNEL0,
	CHANNEL1,
	CHANNEL2,
    CHANNEL3,
	CHANNEL4,
	CHANNEL5,
	CHANNEL6,
	CHANNEL7
	
	
	}CHANNEL_T;
	
	
typedef enum{
	Init_volt_Error,
	Init_Prescaller_Error,
	Init_success,
	conversion_complete=5,
	conversion_NOTcomplete=7,
	Channel_ERROR=9,
	conversion_Time_out
	}ADC_ERROR_T;	 
ADC_ERROR_T ADC_Init(ADC_VoltRef_type volt,ADC_Prescaler_type  scaler);

ADC_ERROR_T ADC_ReadChannel(CHANNEL_T channel );
void ADC_Interrupt_disable(void);
//extra one
ADC_ERROR_T ADC_ReadChannel_Periodic(CHANNEL_T channel ,uint16_t*data);



#endif /* ADC_H_ */