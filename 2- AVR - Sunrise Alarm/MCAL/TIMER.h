/*
 * TIMER.h
 *
 * Created: 8/25/2023 11:38:25 AM
 *  Author: Norhan Mohamed
 */ 


#ifndef TIMER_H_
#define TIMER_H_
/*******************************************************************************
 *                                Includes                                     *
 *******************************************************************************/
#include "STD_types.h"
#include "DIO_interface.h"
#include "ATMega32A_Registers.h"
#include <avr/interrupt.h>

/*******************************************************************************
 *                              Definitions                                    *
 *******************************************************************************/
#define FOC0   DIO_PIN7
#define TOIE0  DIO_PIN0
#define OCIE0  DIO_PIN1
#define TIMER_MAX_REG_VALUE 255
/*******************************************************************************
 *                              Types Definitions                                   *
 *******************************************************************************/
typedef enum{
	NORMAL_MODE,COMPARE_MODE=1, FAST_PWM=9,PHASE_CONTROL=8
}TIMER_modeType;

typedef enum{
	NO_CLOCK_2,F_CPU_CLOCK_2,F_CPU_8_2,F_CPU_32_2,F_CPU_64_2,F_CPU_128_2,F_CPU_256_2,F_CPU_1024_2
}TIMER2_prescaler;

typedef enum{
	NORMAL_OV,TOGGLE,CLEAR,SET_
}TIMER_CompareModeType;

typedef enum{
	NO_MODE,RESERVED,NON_INVERTING,INVERTING
}TIMER_PwmModeType;

typedef enum{
	ErrorInMode, 
	ErrorInPrescaler,
	ErrorInCompareMode,
	ErrorInPwmMode,
	ErrorInCompareValue,
	ErrorInStartValue,
	ErrorInDuty
	}ErrorType;
	
typedef enum {
	SUCCESSFUL,
	UNSUCCESSFUL
	}bool;

typedef struct {
	TIMER_modeType mode;
	TIMER2_prescaler prescaler;
	u8 StartValue;
	u8 CompareValue;
	TIMER_CompareModeType CompareOutput; 
	TIMER_PwmModeType PwmOutput;
	
	}TIMER_ConfigType;
	
typedef enum
{
	TIMER_NO_PRESCALER = 1,
	TIMER_PRESCALER_8 = 2,
	TIMER_PRESCALER_64 = 3,
	TIMER_PRESCALER_256 = 4,
	TIMER_PRESCALER_1024 = 5,
	TIMER0_PRESCALER_TOTAL
} enu_timer0_prescaler_t_;

typedef enum
{
	NORMAL = 0,
	PWM_PHASE_CORRECT,
	CTC,
	PWM_FAST,
	TIMER0_MODES_TOTAL

} enu_timer0_mode_t_;

typedef enum
{
	DISCONNECT_OC0 = 0,
	TOGGLE_OC0,
	CLEAR_OC0,
	SET_OC0,
	TIMER0_OC0_TOTAL
} enu_timer0_oc0_t_;

typedef enum
{
	POLLING = 0,
	OVF_IE,
	CTC_IE,
	TIMER0_INT_EN_TOTAL
} enu_timer0_int_en_t_;

typedef enum
{
	ERROR_TIMER0_OK = 0,
	ERROR_TIMER0_NOK
} enu_timer0_error_status_t;

typedef struct
{
	enu_timer0_prescaler_t_ enu_timer0_prescaler;
	enu_timer0_mode_t_ enu_timer0_mode;
	enu_timer0_int_en_t_ enu_timer0_int_en;
	enu_timer0_oc0_t_ enu_timer0_oc0;
	u8 preload;
	u8 compare_val;
} st_timer0_config_t_;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/
/*
 * @Fn			                        :   TIMER0_Init
 * @brief                               :   Initializes a timer
 *
 * @param[in]   st_arg_timer0_config    :   Pointer to struct that configures the timer
 *             
 * @return      ERROR_TIMER0_OK         :   In case of Successful Operation
 *              ERROR_TIMER0_NOK        :   In case of Failed Operation
 */
enu_timer0_error_status_t TIMER0_Init(st_timer0_config_t_ *st_arg_timer0_config);

/*
 * @Fn			                        :   TIMER0_set_compare
 * @brief                               :   Set the OCR0 value to compare with in CTC mode
 *
 * @param[in]   u8_arg_compare_value    :   the OCR0 value to compare with (0 to 255)
 *             
 * @return      ERROR_TIMER0_OK         :   In case of Successful Operation
 *              ERROR_TIMER0_NOK        :   In case of Failed Operation
 */
enu_timer0_error_status_t TIMER0_set_compare(u8 u8_arg_compare_value);

/*
 * @Fn			                        :   TIMER0_Deinit
 * @brief                               :   De-initializes the timer 
 *
 * @param[in]   void                      
 *
 * @return      void                   
 *                                
 */
void TIMER0_Deinit(void);

/*
 * @Fn			                        :   TIMER0_Init
 * @brief                               :   Initializes a timer
 *
 * @param[in]   st_arg_timer0_config    :   Pointer to struct that configures the timer
 *             
 * @return      SUCCESSFUL              :   In case of Successful Operation
 *              UNSUCCESSFUL            :   In case of Failed Operation
 */
bool TIMER2_Init(const TIMER_ConfigType * Config_ptr);

/*
 * @Fn			                        :   TIMER2_SetDuty
 * @brief                               :   Sets the Duty
 *
 * @param[in]   u8_arg_compare_value    :   Sets the duty of the timer 
 *             
 * @return      ERROR_TIMER0_OK         :   In case of Successful Operation
 *              ERROR_TIMER0_NOK        :   In case of Failed Operation
 */
bool TIMER2_SetDuty(uint8_t duty);

/*
 * @Fn			                        :   TIMER2_setCallBack
 * @brief                               :   Sets the call back function for timer 2 interrupt
 *
 * @param[in]   *a_ptr                  :   Pointer to the callback function
 *             
 * @return      void
 */
void TIMER2_setCallBack(void (*a_ptr)(void));

#endif /* TIMER_H_ */