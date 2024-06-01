#ifndef ULTRASONIC_INTERFACE_H_
#define ULTRASONIC_INTERFACE_H_

#include "stm32f1xx_hal.h"
#include "ultrasonic_confg.h"
#include "app_interface.h"

typedef enum{
Half_way_operation,
Complete_operation=9,
}ULTRASONIC_STAGE;


typedef enum{

ULTRASONIC1=12,
ULTRASONIC2
}ULTRASONIC_NUM;

void Ultrasonic_Updatedistance();
uint16_t  Ultrasonc_getdistace(void);
ULTRASONIC_STAGE Ultrasonc_getstage(void);
void set_Ultrasonic_num(ULTRASONIC_NUM Num);
void Ultrasonic_Int_Timeout(void); //call it when the timeout happen to reset the Ultrasonic to its initial state
#endif //ULTRASONIC_INTERFACE_H_
