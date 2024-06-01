/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BRAKES_Pin GPIO_PIN_14
#define LED_BRAKES_GPIO_Port GPIOC
#define MOTOR_IN_1_RIGHT_Pin GPIO_PIN_2
#define MOTOR_IN_1_RIGHT_GPIO_Port GPIOA
#define MOTOR_IN_2_RIGHT_Pin GPIO_PIN_3
#define MOTOR_IN_2_RIGHT_GPIO_Port GPIOA
#define MOTOR_IN_3_LEFT_Pin GPIO_PIN_4
#define MOTOR_IN_3_LEFT_GPIO_Port GPIOA
#define MOTOR_IN_4_LEFT_Pin GPIO_PIN_5
#define MOTOR_IN_4_LEFT_GPIO_Port GPIOA
#define MOTOR_EN_A_RIGHT_Pin GPIO_PIN_6
#define MOTOR_EN_A_RIGHT_GPIO_Port GPIOA
#define MOTOR_EN_B_LEFT_Pin GPIO_PIN_7
#define MOTOR_EN_B_LEFT_GPIO_Port GPIOA
#define US_TRIGGER_1_DRIVE_Pin GPIO_PIN_0
#define US_TRIGGER_1_DRIVE_GPIO_Port GPIOB
#define US_TRIGGER_2_BACK_Pin GPIO_PIN_1
#define US_TRIGGER_2_BACK_GPIO_Port GPIOB
#define LED_REVERSE_Pin GPIO_PIN_13
#define LED_REVERSE_GPIO_Port GPIOB
#define LED_FRONT_Pin GPIO_PIN_14
#define LED_FRONT_GPIO_Port GPIOB
#define LED_RIGHT_Pin GPIO_PIN_15
#define LED_RIGHT_GPIO_Port GPIOB
#define US_ECHO_1_DRIVE_Pin GPIO_PIN_8
#define US_ECHO_1_DRIVE_GPIO_Port GPIOA
#define US_ECHO_2_BACK_Pin GPIO_PIN_11
#define US_ECHO_2_BACK_GPIO_Port GPIOA
#define LED_LEFT_Pin GPIO_PIN_9
#define LED_LEFT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
