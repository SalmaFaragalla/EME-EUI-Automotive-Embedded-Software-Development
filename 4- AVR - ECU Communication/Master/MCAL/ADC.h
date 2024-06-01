/*
 * ADC.h
 *
 * Created: 8/24/2023 12:04:11 PM
 *  Author: eme
 */

#ifndef ADC_H_
#define ADC_H_

#include <avr/interrupt.h>
#include "ATMega32A_Registers.h"
#include "BIT_MATH.h"

#define Mode interrupt55

#define interrupt55 1
#define polling 2
/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
typedef enum
{
	VREF_AREF,
	VREF_VCC,
	VREF_256

} ADC_VoltRef_type;

typedef enum
{
	ADC_SCALER_2 = 1,
	ADC_SCALER_4,
	ADC_SCALER_8,
	ADC_SCALER_16,
	ADC_SCALER_32,
	ADC_SCALER_64,
	ADC_SCALER_128,
} ADC_Prescaler_type;

typedef enum
{
	CHANNEL0,
	CHANNEL1,
	CHANNEL2,
	CHANNEL3,
	CHANNEL4,
	CHANNEL5,
	CHANNEL6,
	CHANNEL7

} CHANNEL_T;

typedef enum
{
	Init_volt_Error,
	Init_Prescaller_Error,
	Init_success,
	conversion_complete = 5,
	conversion_NOTcomplete = 7,
	Channel_ERROR = 9,
	conversion_Time_out
} ADC_ERROR_T;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/
/*
 * @Fn			                        :   ADC_Init_N
 * @brief                               :   Initializes the ADC
 *
 * @param[in]   volt                    :   ADC reference voltage
 *              scaler                  :   ADC prescaler
 *
 * @return       Init_volt_Error        :   In case there is an error in ADC reference voltage
 *               Init_Prescaller_Error  :   In case there is an error in ADC prescaller
 *               Init_success           :   In case ADC Initialization was successful
 */
ADC_ERROR_T ADC_Init_N(ADC_VoltRef_type volt, ADC_Prescaler_type scaler);

/*
 * @Fn			                        :   ADC_ReadChannel
 * @brief                               :   Reads the ADC channel 
 *
 * @param[in]   volt                    :   ADC reference voltage
 *              scaler                  :   ADC prescaler
 *
 * @return       conversion_NOTcomplete :   In case ADC conversion was not  completed
 *               Channel_ERROR          :   In case there is an error in ADC channel number
 *               conversion_Time_out    :   In case there is a conversion timeout
 */
ADC_ERROR_T ADC_ReadChannel(CHANNEL_T channel);

/*
 * @Fn			                        :   ADC_Interrupt_disable
 * @brief                               :   Disables the ADC Interrupt
 *
 * @param[in]   void                      
 *
 * @return      void                   
 *                                
 */
void ADC_Interrupt_disable(void);

/*
 * @Fn			                        :   ADC_ReadChannel_Periodic
 * @brief                               :   Reads the ADC channel
 *
 * @param[in]   volt                    :   ADC reference voltage
 *              scaler                  :   ADC prescaler
 *
 * @return       conversion_NOTcomplete :   In case ADC conversion was not  completed
 *               Channel_ERROR          :   In case there is an error in ADC channel number
 *               conversion_Time_out    :   In case there is a conversion timeout
 */
ADC_ERROR_T ADC_ReadChannel_Periodic(CHANNEL_T channel, u16 *data);

#endif /* ADC_H_ */
