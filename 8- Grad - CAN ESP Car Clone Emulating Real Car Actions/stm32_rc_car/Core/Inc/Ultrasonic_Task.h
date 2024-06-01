#ifndef ULTRASONIC_TASK_H_
#define ULTRASONIC_TASK_H_

#include "ultrasonic_interface.h"
typedef enum{
	Green_Flag,
	Red_Flag
}ULTRASONIC_PASS;

typedef enum{
	Success_=8,
	Time_Out_=11
}ULTRASONIC_Status;

void Ultrasonic_Task (void*pvParameter );
void Ultrasonic_Timeout_Task(void*pvParameter);
void Ultrasonictest_Task(void*pvParameter);

#endif //ULTRASONIC_TASK_H_
