/*
 * LED.h
 *
 * Created: 20-Aug-23 10:36:25 AM
 *  Author: Salma
 */ 


#ifndef LED_H_
#define LED_H_


#include "LED_Cfg.h"
#include "STD_types.h"
#include "Timer.h"

#define LED_GREEN   0
#define LED_BLUE    1
#define LED_YELLOW  2
#define LED_RED     3


void LED_Init(void);
void LED_TurnOn(uint8_t led_color);
void LED_TurnOff(uint8_t led_color);
void LED_Blink_5sec(uint8_t led_color);
void LED_deinit(uint8_t led_color);

#endif /* LED_H_ */