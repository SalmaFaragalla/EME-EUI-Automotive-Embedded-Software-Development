/*
 * ADC.c
 *
 * Created: 8/24/2023 12:03:54 PM
 *  Author: eme
 */
#include "ADC.h"

volatile u16 Adc_Data;

ADC_ERROR_T ADC_Init_N(ADC_VoltRef_type volt, ADC_Prescaler_type scaler)
{
	ADC_ERROR_T status;
	if (volt > 2 || volt < 0)
	{
		status = Init_volt_Error;
		return status;
	}
	else
	{

		switch (volt) // select volt
		{

		case VREF_256:
			SET_BIT(ADMUX, REFS0);
			SET_BIT(ADMUX, REFS1);
			break;
		case VREF_AREF:
			CLR_BIT(ADMUX, REFS0);
			CLR_BIT(ADMUX, REFS1);
			break;
		case VREF_VCC:
			SET_BIT(ADMUX, REFS0);
			CLR_BIT(ADMUX, REFS1);
			break;
		}
	}

	ADCSRA = ADCSRA & 0xf8; // 0b11111000
	if (scaler > 7)
	{
		status = Init_Prescaller_Error;
		return status;
	}
	else
	{
		scaler = scaler & 0x07;
		ADCSRA = ADCSRA | scaler;
		/*ADC enable*/
		SET_BIT(ADCSRA, ADEN);
		status = Init_success;
	}

	return status;
}

// polling busy_wiat
#if Mode == polling
ADC_ERROR_T ADC_ReadChannel(CHANNEL_T channel)
{

	ADC_ERROR_T status;
	u16 count = 0;
	ADMUX = ADMUX & 0Xe0;
	if (channel > 7)
	{
		status = Channel_ERROR;
	}
	else
	{
		ADMUX |= channel;

		SET_BIT(ADCSRA, ADSC);
		while (GET_BIT(ADCSRA, ADSC))
		{
			count++;
			if (count > 1000)
			{
				status = conversion_Time_out;
				return status;
			}
		}
		SET_BIT(ADCSRA, ADIF);
		Adc_Data = ADC;
		status = conversion_complete;

		return status;
	}
}
/*interrupt*/
#elif Mode == interrupt55
ADC_ERROR_T ADC_ReadChannel(CHANNEL_T channel)
{
  
    sei();  /*SF*/
    
 
	ADC_ERROR_T status;
	ADMUX = ADMUX & 0Xe0;
	if (channel > 7)
	{
		status = Channel_ERROR;
	}
	else
	{

		ADMUX = ADMUX & 0Xe0;
		ADMUX |= channel;
		SET_BIT(ADCSRA, ADSC);
		SET_BIT(ADCSRA, ADIE);
		status = conversion_complete;
	}
	return status;
}
#endif
void ADC_Interrupt_disable(void)
{

	CLR_BIT(ADCSRA, ADIE);
}

/*polling Periodic check*/
ADC_ERROR_T ADC_ReadChannel_Periodic(CHANNEL_T channel, u16 *data)
{


	ADC_ERROR_T status;
	ADMUX = ADMUX & 0Xe0;
	if (channel)
		ADMUX |= channel;

	SET_BIT(ADCSRA, ADSC);

	if (!GET_BIT(ADCSRA, ADSC))
	{
		*data = ADC;
		status = conversion_complete;
	}
	else
	{
		status = conversion_NOTcomplete;
	}
	CLR_BIT(ADCSRA, ADIF);
	return status;
    
}


ISR(ADC_vect)
{

	Adc_Data = ((u16)ADC * 100) / 1023;
}
