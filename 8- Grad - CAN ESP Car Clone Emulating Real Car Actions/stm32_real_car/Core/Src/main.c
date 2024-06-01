/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
CAN_FilterTypeDef FilterConfig;
CAN_RxHeaderTypeDef RxHeader; //structure for message reception

/* Create semaphore for UART transmit task */
//SemaphoreHandle_t uartTransmitRequestSemaphore;

/* Create uart transmit queue */
QueueHandle_t canProcessQueue;
QueueHandle_t uartTransmitQueue;

static st_last_data_state_t st_gs_last_data_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void task_process_can_data(void * pvParameters);
void task_uart_tx_data(void * pvParameters);
static void helper_check_and_update_reverse_lights_based_on_transmission(uint8_t u8_a_new_transmission_value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	st_can_queued_item_t st_can_queued_item;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* check message */
	/* Read Message into local data converter rx buffer */
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, st_can_queued_item.un_data_converter.RxData);

	/* store can id for further processing */
	st_can_queued_item.item_id = RxHeader.StdId;

    /* Toggle CAN RX LED */
    HAL_GPIO_TogglePin(CAN_RX_LED_ARGS);

    /* Queue can data for processing */
    xQueueSendToBackFromISR(canProcessQueue, &st_can_queued_item, &xHigherPriorityTaskWoken);

}

/* Inline helping function(s) */
static inline uint8_t app_calc_throttle_power_percentage(uint16_t u16_a_throttle_reading)
{
    uint8_t uint8_l_retval = ZERO;

    /* calculate throttle percentage */
    uint8_l_retval = (u16_a_throttle_reading * MAX_PERCENTAGE) / (THROTTLE_READING_REDUCTION_FACTOR);

    if(APP_ESP_DATA_THROTTLE_LEVEL_4_MAX < u16_a_throttle_reading)
    {
        /* invalid data range - do nothing */
    }
    else if(u16_a_throttle_reading >= APP_ESP_DATA_THROTTLE_LEVEL_4_MIN)
    {
        /* throttle level 4 (max) */
        uint8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_THROTTLE, APP_ESP_DATA_THROTTLE_LEVEL_4_MAP_VAL);
    }
    else if(u16_a_throttle_reading >= APP_ESP_DATA_THROTTLE_LEVEL_3_MIN)
    {
        /* throttle level 3 (max) */
        uint8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_THROTTLE, APP_ESP_DATA_THROTTLE_LEVEL_3_MAP_VAL);
    }
    else if(u16_a_throttle_reading >= APP_ESP_DATA_THROTTLE_LEVEL_2_MIN)
    {
        /* throttle level 2 (max) */
        uint8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_THROTTLE, APP_ESP_DATA_THROTTLE_LEVEL_2_MAP_VAL);
    }
    else if(u16_a_throttle_reading >= APP_ESP_DATA_THROTTLE_LEVEL_1_MIN)
    {
        /* throttle level 1 (max) */
        uint8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_THROTTLE, APP_ESP_DATA_THROTTLE_LEVEL_1_MAP_VAL);
    }
    else if(u16_a_throttle_reading >= APP_ESP_DATA_THROTTLE_STOP_MIN)
    {
        /* throttle stop level (min) */
        uint8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_THROTTLE, APP_ESP_DATA_THROTTLE_STOP_MAP_VAL);
    }


    return uint8_l_retval;
}

static void helper_check_and_update_reverse_lights_based_on_transmission(uint8_t u8_a_new_transmission_value)
{
    uint8_t u8_l_reverse_lights = ZERO;

    if(TRANSMISSION_STATE_REVERSE == u8_a_new_transmission_value)
    {
        /* queue reverse lights */
        u8_l_reverse_lights = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_REVERSE, ENABLED);
        xQueueSendToBack(uartTransmitQueue, &u8_l_reverse_lights, portMAX_DELAY);
    }
    else
    {
        /* Check if reverse lights are ON, turn them off */
        if(TRANSMISSION_STATE_REVERSE == st_gs_last_data_state.en_transmission_state)
        {
            /* queue reverse lights */
            u8_l_reverse_lights = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_REVERSE, DISABLED);
            xQueueSendToBack(uartTransmitQueue, &u8_l_reverse_lights, portMAX_DELAY);
        }
        else
        {
            /* Do Nothing */
        }
    }
}

/* Map transmission state */
static inline uint8_t app_map_transmission(uint8_t u8_a_new_transmission_value)
{
    uint8_t u8_l_retval = ZERO;

    switch(u8_a_new_transmission_value)
    {
        case APP_CAR_TRANSMISSION_PARK:
        {
            if(TRANSMISSION_STATE_PARKING != st_gs_last_data_state.en_transmission_state)
            {
                /* check and update reverse lights based on new transmission value */
                helper_check_and_update_reverse_lights_based_on_transmission(TRANSMISSION_STATE_PARKING);

                /* update global state */
                st_gs_last_data_state.en_transmission_state = TRANSMISSION_STATE_PARKING;

                /* generate frame */
                u8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_TRANSMISSION, SHIFT_HIGH_NIBBLE_TO_LOW(u8_a_new_transmission_value));
            }
            else
            {
                /* duplicated data - ignore */
            }

            break;
        }
        case APP_CAR_TRANSMISSION_DRIVE:
        {
            if(TRANSMISSION_STATE_DRIVE != st_gs_last_data_state.en_transmission_state)
            {
                /* check and update reverse lights based on new transmission value */
                helper_check_and_update_reverse_lights_based_on_transmission(TRANSMISSION_STATE_DRIVE);

                /* update global state */
                st_gs_last_data_state.en_transmission_state = TRANSMISSION_STATE_DRIVE;

                /* generate frame */
                u8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_TRANSMISSION, SHIFT_HIGH_NIBBLE_TO_LOW(u8_a_new_transmission_value));
            }
            else
            {
                /* duplicated data - ignore */
            }

            break;
        }
        case APP_CAR_TRANSMISSION_NEUTRAL:
        {
            if(TRANSMISSION_STATE_NEUTRAL != st_gs_last_data_state.en_transmission_state)
            {
                /* check and update reverse lights based on new transmission value */
                helper_check_and_update_reverse_lights_based_on_transmission(TRANSMISSION_STATE_NEUTRAL);

                /* update global state */
                st_gs_last_data_state.en_transmission_state = TRANSMISSION_STATE_NEUTRAL;

                /* generate frame */
                u8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_TRANSMISSION, SHIFT_HIGH_NIBBLE_TO_LOW(u8_a_new_transmission_value));
            }
            else
            {
                /* duplicated data - ignore */
            }

            break;
        }
        case APP_CAR_TRANSMISSION_REVERSE:
        {
            if(TRANSMISSION_STATE_REVERSE != st_gs_last_data_state.en_transmission_state)
            {
                /* check and update reverse lights based on new transmission value */
                helper_check_and_update_reverse_lights_based_on_transmission(TRANSMISSION_STATE_REVERSE);

                /* update global state */
                st_gs_last_data_state.en_transmission_state = TRANSMISSION_STATE_REVERSE;

                /* generate frame */
                u8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_TRANSMISSION, SHIFT_HIGH_NIBBLE_TO_LOW(u8_a_new_transmission_value));
            }
            else
            {
                /* duplicated data - ignore */
            }

            break;
        }

        default:
        {
            /* Do Nothing */
            break;
        }
    }

    return u8_l_retval;
}

/* Map Steering Value */
static inline uint8_t app_map_steering(uint8_t u8_a_steering_val)
{
    uint8_t u8_l_retval = ZERO;

    en_steering_state_t en_l_current_steering = STEERING_STATE_NONE;

    /* check steering range */
    /* invalid boundary check */
    if(
            (u8_a_steering_val > APP_CAR_STEERING_THRESHOLD_SHARP_LEFT_MAX) ||
            (u8_a_steering_val < APP_CAR_STEERING_THRESHOLD_SHARP_RIGHT_MIN)
            )
    {
        /* Invalid data - ignore */
    }
        /* decremental steering check */
    else if(u8_a_steering_val >= APP_CAR_STEERING_THRESHOLD_SHARP_LEFT_MIN)
    {
        /* sharp left */
        en_l_current_steering = STEERING_STATE_SHARP_LEFT;
    }
    else if(u8_a_steering_val >= APP_CAR_STEERING_THRESHOLD_LEFT_MIN)
    {
        /* slight left */
        en_l_current_steering = STEERING_STATE_LEFT;
    }
    else if(u8_a_steering_val >= APP_CAR_STEERING_THRESHOLD_STRAIGHT_MIN)
    {
        /* straight - no steering */
        en_l_current_steering = STEERING_STATE_STRAIGHT;
    }
    else if(u8_a_steering_val >= APP_CAR_STEERING_THRESHOLD_RIGHT_MIN)
    {
        /* slight right */
        en_l_current_steering = STEERING_STATE_RIGHT;
    }
    else
    {
        /* sharp right */
        en_l_current_steering = STEERING_STATE_SHARP_RIGHT;
    }

    /* queue sending signal to ESP */
    if(
            (en_l_current_steering != STEERING_STATE_NONE) &&
            (en_l_current_steering != st_gs_last_data_state.en_steering_state)
            )
    {
        /* update last steering state */
        st_gs_last_data_state.en_steering_state = en_l_current_steering;

        /* send data */
        u8_l_retval = GENERATE_ESP_FRAME(APP_ESP_HEADER_STEERING, en_l_current_steering);
    }

    return u8_l_retval;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

//  uint8_t * namePtr = NULL;
//  namePtr = myname;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* Create semaphore for UART transmit task */
//  uartTransmitRequestSemaphore = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  canProcessQueue = xQueueCreate(APP_CAN_PROCESS_QUEUE_LENGTH, sizeof(st_can_queued_item_t));
  uartTransmitQueue = xQueueCreate(APP_UART_TX_QUEUE_LENGTH, sizeof(app_uart_queue_item_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  xTaskCreate(task_process_can_data, "can2rc", APP_RTOS_TASK_STACK_SIZE, (void *) NULL, 0, (void *) NULL);
  xTaskCreate(task_uart_tx_data, "uart-tx", APP_RTOS_TASK_STACK_SIZE, (void *) NULL, 0, (void *) NULL);

  /* Setup default values for global variables */
    st_gs_last_data_state.en_steering_state = STEERING_STATE_NONE;
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* do nothing */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* Configure CAN Receiving Filter */
    /* set FIFO assignment */
    FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    FilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;
    /* 0x245<<5; the ID that the filter looks for: Zero to pass all IDs */
    FilterConfig.FilterIdHigh       =   CAN_FILTER_ID_HIGH      ;
    FilterConfig.FilterIdLow        =   CAN_FILTER_ID_LOW       ;
    FilterConfig.FilterMaskIdHigh   =   CAN_FILTER_MASK_ID_HIGH ;
    FilterConfig.FilterMaskIdLow    =   CAN_FILTER_MASK_ID_LOW  ;

    /* Set Filter Scale */
    FilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; //set filter scale

    /* Enable Filter */
    FilterConfig.FilterActivation = ENABLE;

    /* Configure CAN Filter */
    HAL_CAN_ConfigFilter(&hcan, &FilterConfig); //configure CAN filter

    /* Start CAN */
    HAL_CAN_Start(&hcan);

    /* Enable Rx FIFO0 Interrupt */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);


  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void task_uart_tx_data(void * pvParameters)
{
    app_uart_queue_item_t app_uart_queue_item;

    HAL_StatusTypeDef uartTxStatus = HAL_OK;

    /* Task main loop */
    for(;;)
    {
        xQueueReceive(uartTransmitQueue, &app_uart_queue_item, portMAX_DELAY);

        /* transmit over uart */
        uartTxStatus = HAL_UART_Transmit(&huart3, &app_uart_queue_item, 1, APP_UART_TX_TIMEOUT_MS);

        if(HAL_OK == uartTxStatus)
        {
            /* Toggle UART TX LED */
            HAL_GPIO_TogglePin(UART_TX_LED_ARGS);
        }
        else
        {
            /* Do Nothing */
            /* Re-queue to front of queue */
            xQueueSendToFront(uartTransmitQueue, &app_uart_queue_item, portMAX_DELAY);
        }
    }
}

void task_process_can_data(void * pvParameters)
{
    /* Define queue item variable */
    st_can_queued_item_t st_l_can_queue_item;

    /*Define byte to be sent over UART */
    uint8_t u8_l_uart_tx_data = ZERO;

    union
    {
        uint32_t			u32_lights_val;
        st_lighting_bits_t	st_lights_bits;
    }un_l_lights_conv;

    /* Task main loop */
	for(;;)
	{
        /* reset local variables */
        u8_l_uart_tx_data = ZERO;

		/* Dequeue / block until more data is available to dequeue */
		xQueueReceive(canProcessQueue, &st_l_can_queue_item, portMAX_DELAY);

		/* Map Data */
		switch (st_l_can_queue_item.item_id)
		{
			case APP_CAN_ID_THROTTLE:
			{
				/* Map throttle values to 0 -> 100% */
                u8_l_uart_tx_data = app_calc_throttle_power_percentage(st_l_can_queue_item.un_data_converter.u16_rxNumber);

                /* check not to send duplicated data */
                if(u8_l_uart_tx_data != st_gs_last_data_state.u8_throttle_val)
                {
                    st_gs_last_data_state.u8_throttle_val = u8_l_uart_tx_data;
                    /* queue data to be sent over UART */
                    xQueueSendToBack(uartTransmitQueue, &u8_l_uart_tx_data, portMAX_DELAY);
                }
                else
                {
                    /* Duplicate data - do nothing */
                }

                break;
			}
			case APP_CAN_ID_STEERING:
			{
				/* Map steering values */
				u8_l_uart_tx_data = app_map_steering(st_l_can_queue_item.un_data_converter.u8_rxNumber);

                /* Check valid mapping */
                if(APP_ESP_HEADER_STEERING == SHIFT_HIGH_NIBBLE_TO_LOW(u8_l_uart_tx_data))
                {
                    /* queue data to be sent over UART */
                    xQueueSendToBack(uartTransmitQueue, &u8_l_uart_tx_data, portMAX_DELAY);
                }
                else
                {
                    /* invalid data - do nothing */
                }

                break;
			}

			case APP_CAN_ID_LIGHTS:
			{
				/* check if there's a change */
				if(st_l_can_queue_item.un_data_converter.u32_rxNumber != st_gs_last_data_state.u32_lighting_val)
				{
					/* changed - calculate changes */
					un_l_lights_conv.u32_lights_val = GET_CHANGED_BITS(st_gs_last_data_state.u32_lighting_val,
                                                                       st_l_can_queue_item.un_data_converter.u32_rxNumber);

					/* update last state global variable */
					st_gs_last_data_state.u32_lighting_val = st_l_can_queue_item.un_data_converter.u32_rxNumber;

					/* check changed lights */

                    /* Brakes light */
					if(un_l_lights_conv.st_lights_bits.u32_brake_lights_bit)
					{
						/* brake lights state changed - toggle last state */
                        st_gs_last_data_state.st_lighting_state.bool_brake_lights = st_l_can_queue_item.un_data_converter.u8_lighting_bits.u32_brake_lights_bit;

                        if(TRUE == st_gs_last_data_state.st_lighting_state.bool_brake_lights)
                        {
                            st_gs_last_data_state.u8_throttle_val = GENERATE_ESP_FRAME(APP_ESP_HEADER_THROTTLE, APP_ESP_DATA_THROTTLE_STOP_MAP_VAL);
                        }
                        else
                        {
                            /* Keep last throttle value in memory */
                        }

						/* queue to ESP */
						u8_l_uart_tx_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_BRAKES, st_gs_last_data_state.st_lighting_state.bool_brake_lights);

                        xQueueSendToBack(uartTransmitQueue, &u8_l_uart_tx_data, portMAX_DELAY);
					}
                    /* Left Indicator */
					if(un_l_lights_conv.st_lights_bits.u32_left_indicator_bit)
					{
						/* left indicator lights state changed - toggle last state */
                        st_gs_last_data_state.st_lighting_state.bool_left_indicator = st_l_can_queue_item.un_data_converter.u8_lighting_bits.u32_left_indicator_bit;

                        /* queue to ESP */
                        u8_l_uart_tx_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_L_INDICATORS, st_gs_last_data_state.st_lighting_state.bool_left_indicator);
                        xQueueSendToBack(uartTransmitQueue, &u8_l_uart_tx_data, portMAX_DELAY);
					}
                    /* Right Indicator */
					if(un_l_lights_conv.st_lights_bits.u32_right_indicator_bit)
					{
						/* right indicator lights state changed - update last state */
                        st_gs_last_data_state.st_lighting_state.bool_right_indicator = st_l_can_queue_item.un_data_converter.u8_lighting_bits.u32_right_indicator_bit;

                        /* queue to ESP */
                        u8_l_uart_tx_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_R_INDICATORS, st_gs_last_data_state.st_lighting_state.bool_right_indicator);
                        xQueueSendToBack(uartTransmitQueue, &u8_l_uart_tx_data, portMAX_DELAY);
                    }
                    /* Front Lights */
					if(un_l_lights_conv.st_lights_bits.u32_front_light_bit)
					{
						/* front lights state changed - update last state */
						st_gs_last_data_state.st_lighting_state.bool_front_lights = st_l_can_queue_item.un_data_converter.u8_lighting_bits.u32_front_light_bit;

                        /* queue to ESP */
                        u8_l_uart_tx_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_FRONT, st_gs_last_data_state.st_lighting_state.bool_front_lights);
                        xQueueSendToBack(uartTransmitQueue, &u8_l_uart_tx_data, portMAX_DELAY);
                    }

				}
				else
				{
					/* no change - do nothing */
				}



				break;
			}
			case APP_CAN_ID_TRANSMISSION:
			{
				/* check transmission state */
				u8_l_uart_tx_data = app_map_transmission(st_l_can_queue_item.un_data_converter.u8_rxNumber);

                /* check valid mapping */
                if(SHIFT_HIGH_NIBBLE_TO_LOW(u8_l_uart_tx_data) == APP_ESP_HEADER_TRANSMISSION)
                {
                    /* queue data to be sent over UART */
                    xQueueSendToBack(uartTransmitQueue, &u8_l_uart_tx_data, portMAX_DELAY);
                }
                else
                {
                    /* Do Nothing */
                }
				break;
			}

			default:
			{
				/* Do Nothing */
				break;
			}
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
