/*
 * simple_RTOS.c
 *
 *  Created on: Oct 13, 2023
 *      Author: Salma
 */

#include "simple_RTOS.h"

task_t TasksArray [NUM_OF_TASKS];

void BlinkRed (void)
{
	HAL_GPIO_TogglePin(LED_RED_ARG);
}
void BlinkYellow (void)
{
	HAL_GPIO_TogglePin(LED_YELLOW_ARG);
}
void BlinkGreen (void)
{
	HAL_GPIO_TogglePin(LED_GREEN_ARG);
}
void CreateTask (uint8_t priority , uint16_t periodicity , void (*Fptr) (void))
{
	TasksArray[priority].periodicity = periodicity ;
	TasksArray[priority].Fptr = Fptr;
}
void Scheduler (void)
{

	uint8_t taskIndex = 0;
	for (taskIndex = 0 ; taskIndex < NUM_OF_TASKS ; ++taskIndex)
	{
		if ((uwTick  % TasksArray[taskIndex].periodicity) == 0)
		{
			TasksArray[taskIndex].Fptr();
			if (taskIndex == NUM_OF_TASKS-1)
			{
				uwTick = 0;
			}
		}
	}
}
