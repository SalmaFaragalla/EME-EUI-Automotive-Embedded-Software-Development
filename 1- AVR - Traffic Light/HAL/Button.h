/*
 * Button.h
 *
 * Created: 8/20/2023 11:05:45 AM
 *  Author: nora
 */ 


#ifndef BUTTON_H_
#define BUTTON_H_

#include "STD_types.h"
#include "Traffic_Light.h"

void Button_Init(uint8_t Port , uint8_t Pin);
void Button_Pressed(void);



#endif /* BUTTON_H_ */