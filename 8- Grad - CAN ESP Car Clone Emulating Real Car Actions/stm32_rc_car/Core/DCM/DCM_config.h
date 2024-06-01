/**
 ******************************************************************************
 * @file    DCM_config.h
 * @author  Salma Faragalla
 * @brief   DCM module configuration file.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DCM_DCM_CONFIG_H_
#define DCM_DCM_CONFIG_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported constants --------------------------------------------------------*/

/* Left motor configurations */
#define DCM_LEFT_PORT (GPIOA)
#define DCM_LEFT_IN1  (GPIO_PIN_4)
#define DCM_LEFT_IN2  (GPIO_PIN_5)
#define DCM_LEFT_EN   (GPIO_PIN_7)

/* Right motor configurations */
#define DCM_RIGHT_PORT (GPIOA)
#define DCM_RIGHT_IN1  (GPIO_PIN_2)
#define DCM_RIGHT_IN2  (GPIO_PIN_3)
#define DCM_RIGHT_EN   (GPIO_PIN_6)

#endif /* DCM_DCM_CONFIG_H_ */
