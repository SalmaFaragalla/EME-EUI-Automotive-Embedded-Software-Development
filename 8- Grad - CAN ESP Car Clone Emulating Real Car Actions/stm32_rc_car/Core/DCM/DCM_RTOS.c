/**
 ******************************************************************************
 * @file    DCM_RTOS.c
 * @author  Salma Faragalla
 * @brief   DCM task in FreeRTOS kernel.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "DCM_RTOS.h"

/* Private variables ---------------------------------------------------------*/
st_dcm_config_t DCM_Left_config = {DCM_LEFT_PORT, DCM_LEFT_IN1, DCM_LEFT_IN2, &htim3, TIM_CHANNEL_2};
st_dcm_config_t DCM_Right_config = {DCM_RIGHT_PORT, DCM_RIGHT_IN1, DCM_RIGHT_IN2, &htim3, TIM_CHANNEL_1};

/* Private function prototypes -----------------------------------------------*/
static void DCM_CheckSpeed(uint8_t a_throttle_position);
static void DCM_CheckSteeringForward(uint8_t a_steering_position);
static void DCM_CheckSteeringBackward(uint8_t a_steering_position);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  checks the throttle position and set the speed accordingly.
 * @param  a_throttle_position: throttle position
 *          This parameter can be one of the following values:
 *            @arg SPEED_ZERO: Zero speed
 *            @arg SPEED_MIN:  Minimum speed
 *            @arg SPEED_MED:  Medium speed
 *            @arg SPEED_HIGH: High speed
 *            @arg SPEED_MAX:  Maximum speed
 * @retval None
 */
static void DCM_CheckSpeed(uint8_t a_throttle_position)
{
    switch (a_throttle_position)
    {
        case SPEED_ZERO:
        {
            DCM_SetSpeed(&DCM_Left_config, SPEED_ZERO);
            DCM_SetSpeed(&DCM_Right_config, SPEED_ZERO);
            break;
        }
        case SPEED_MIN:
        {
            DCM_SetSpeed(&DCM_Left_config, SPEED_MIN);
            DCM_SetSpeed(&DCM_Right_config, SPEED_MIN);
            break;
        }
        case SPEED_MED:
        {
            DCM_SetSpeed(&DCM_Left_config, SPEED_MED);
            DCM_SetSpeed(&DCM_Right_config, SPEED_MED);
            break;
        }
        case SPEED_HIGH:
        {
            DCM_SetSpeed(&DCM_Left_config, SPEED_HIGH);
            DCM_SetSpeed(&DCM_Right_config, SPEED_HIGH);
            break;
        }
        case SPEED_MAX:
        {
            DCM_SetSpeed(&DCM_Left_config, SPEED_MAX);
            DCM_SetSpeed(&DCM_Right_config, SPEED_MAX);
            break;
        }
        default:
        {
            /*Do Nothing*/
            break;
        }
    }
}

/**
 * @brief  checks the steering wheel position and set the motors forward direction accordingly.
 * @param  a_steering_position: steering wheel position
 *          This parameter can be one of the following values:
 *            @arg STEERING_STRAIGHT:    Steering wheel is straight
 *            @arg STEERING_RIGHT:       Steering wheel is turned to the right
 *            @arg STEERING_LEFT:     	  Steering wheel is turned to the left
 *            @arg STEERING_SHARP_RIGHT: Steering wheel is turned to the right sharply
 *            @arg STEERING_SHARP_LEFT:  Steering wheel is turned to the left sharply
 * @retval None
 */
static void DCM_CheckSteeringForward(uint8_t a_steering_position)
{
    switch (a_steering_position)
    {
        case STEERING_STRAIGHT:
        {
            DCM_MoveForward(&DCM_Left_config);
            DCM_MoveForward(&DCM_Right_config);
            break;
        }
        case STEERING_RIGHT:
        {
            DCM_MoveRightForward(&DCM_Left_config, &DCM_Right_config);
            break;
        }
        case STEERING_LEFT:
        {
            DCM_MoveLeftForward(&DCM_Left_config, &DCM_Right_config);
            break;
        }
        case STEERING_SHARP_RIGHT:
        {
            DCM_MoveRightSharpForward(&DCM_Left_config, &DCM_Right_config);
            break;
        }
        case STEERING_SHARP_LEFT:
        {
            DCM_MoveLeftSharpForward(&DCM_Left_config, &DCM_Right_config);
            break;
        }
        default:
        {
            /*Do Nothing*/
            break;
        }
    }
}

/**
 * @brief  checks the steering wheel position and set the motors backward direction accordingly.
 * @param  a_steering_position: steering wheel position
 *          This parameter can be one of the following values:
 *            @arg STEERING_STRAIGHT:    Steering wheel is straight
 *            @arg STEERING_RIGHT:       Steering wheel is turned to the right
 *            @arg STEERING_LEFT:     	  Steering wheel is turned to the left
 *            @arg STEERING_SHARP_RIGHT: Steering wheel is turned to the right sharply
 *            @arg STEERING_SHARP_LEFT:  Steering wheel is turned to the left sharply
 * @retval None
 */
static void DCM_CheckSteeringBackward(uint8_t a_steering_position)
{
    switch (a_steering_position)
    {
        case STEERING_STRAIGHT:
        {
            DCM_MoveBackward(&DCM_Left_config);
            DCM_MoveBackward(&DCM_Right_config);
            break;
        }
        case STEERING_RIGHT:
        {
            DCM_MoveRightBackward(&DCM_Left_config, &DCM_Right_config);
            break;
        }
        case STEERING_LEFT:
        {
            DCM_MoveLeftBackward(&DCM_Left_config, &DCM_Right_config);
            break;
        }
        case STEERING_SHARP_RIGHT:
        {
            DCM_MoveRightSharpBackward(&DCM_Left_config, &DCM_Right_config);
            break;
        }
        case STEERING_SHARP_LEFT:
        {
            DCM_MoveLeftSharpBackward(&DCM_Left_config, &DCM_Right_config);
            break;
        }
        default:
        {
            /*Do Nothing*/
            break;
        }
    }
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  checks the gear, throttle and steering wheel positions and moves the motors accordingly.
 * @param  pvParameters: value that is passed as the parameter to the created task.
 * @retval None
 */
void Task_DCM(void *pvParameters)
{
    for (;;)
    {

        if (xSemaphoreTake(GEAR_SEMAPHORE, portMAX_DELAY) == pdTRUE || xSemaphoreTake(STEERING_SEMAPHORE, portMAX_DELAY) == pdTRUE)
        {

            switch (GEAR_POSITION)
            {
                case GEAR_DRIVE:
                {
                    DCM_CheckSpeed(THROTTLE_POSITION);
                    DCM_CheckSteeringForward(STEERING_WHEEL_POSITION);
                    break;
                }
                case GEAR_NEUTRAL:
                {
                    DCM_Stop(&DCM_Left_config);
                    DCM_Stop(&DCM_Right_config);
                    break;
                }
                case GEAR_PARKING:
                {
                    DCM_Stop(&DCM_Left_config);
                    DCM_Stop(&DCM_Right_config);
                    break;
                }
                case GEAR_REVERSE:
                {
                    DCM_CheckSpeed(THROTTLE_POSITION);
                    DCM_CheckSteeringBackward(STEERING_WHEEL_POSITION);
                    break;
                }
                default:
                {
                    /*Do Nothing*/
                    break;
                }
            }
        }
    }
}
