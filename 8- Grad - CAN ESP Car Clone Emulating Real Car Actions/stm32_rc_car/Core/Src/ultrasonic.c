
#include "ultrasonic_interface.h"
#include "Ultrasonic_Task.h"
#include "stm32f1xx_hal.h"
 extern TIM_HandleTypeDef htim1;
 extern void delay_us(uint16_t us );
 static ULTRASONIC_STAGE Ultrasonic_stage;
 static ULTRASONIC_NUM Number_ultrasonic;
 static uint8_t ULTRASONIC_FLAG;
 static uint16_t distance;

void delay_us (uint16_t us)
    {
        __HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
        while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
    }


void Ultrasonic_Int_Timeout() // Call it when The timeout happen
    {
        if(Number_ultrasonic ==ULTRASONIC1)
        {
            TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_CC1);
        }
        else if(Number_ultrasonic ==ULTRASONIC2)
        {
            TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_CC4);
        }
        ULTRASONIC_FLAG=0;
    }
 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
    {

	 static uint16_t TIM1_ULTRA;
	 static uint16_t TIM2_ULTRA;
	 static uint16_t DIFFERNCE;

        if(ULTRASONIC_FLAG==0)
        {
            if(Number_ultrasonic ==ULTRASONIC1)
            {
                TIM1_ULTRA= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1 );
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            else if(Number_ultrasonic==ULTRASONIC2)
            {
                TIM1_ULTRA= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4 );
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_4,TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            ULTRASONIC_FLAG=1;
            Ultrasonic_stage=Half_way_operation;
        }
        else if(ULTRASONIC_FLAG==1)
        {
            if(Number_ultrasonic==ULTRASONIC1)
            {
                TIM2_ULTRA= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1 );
                __HAL_TIM_SET_COUNTER(htim,0);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
                __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_CC1);
            }
            else if(Number_ultrasonic ==ULTRASONIC2)
            {

                TIM2_ULTRA= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4 );

                __HAL_TIM_SET_COUNTER(htim,0);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
                __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_CC4);
            }


            ULTRASONIC_FLAG=2;
            Ultrasonic_stage=Complete_operation;
        }

        if(ULTRASONIC_FLAG==2)
        {
            if(TIM2_ULTRA>TIM1_ULTRA)
            {
                DIFFERNCE=TIM2_ULTRA-TIM1_ULTRA;
            }
            else
            {
                DIFFERNCE=(0xffff-TIM1_ULTRA)+TIM2_ULTRA;
            }
            distance = DIFFERNCE* 0.034/2;
            ULTRASONIC_FLAG=0;
        }


        else
        {
            //donthing

        }

    }


void Ultrasonic_Updatedistance(void)
    {
        switch	(Number_ultrasonic)
        {
        case ULTRASONIC1:
            HAL_GPIO_WritePin(TRIGER_FORWARD_PORT,TRIGER_FORWARD_PIN, GPIO_PIN_SET);
            delay_us(10);
            HAL_GPIO_WritePin(TRIGER_FORWARD_PORT, TRIGER_FORWARD_PIN, GPIO_PIN_RESET);
            __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_CC1);
            break;
        case ULTRASONIC2:
            HAL_GPIO_WritePin(TRIGER_BACKWARD_PORT, TRIGER_BACKWARD_PIN, GPIO_PIN_SET);
            delay_us(10);
            HAL_GPIO_WritePin(TRIGER_BACKWARD_PORT,TRIGER_BACKWARD_PIN, GPIO_PIN_RESET);
            __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_CC4);

            break;
        default:
            break;
        }

    }

void set_Ultrasonic_num(ULTRASONIC_NUM Num)
    {
        Number_ultrasonic=Num;
    }

uint16_t  Ultrasonc_getdistace(void)
    {

        return  distance;

    }

ULTRASONIC_STAGE Ultrasonc_getstage(void)
    {

        return Ultrasonic_stage;
    }





