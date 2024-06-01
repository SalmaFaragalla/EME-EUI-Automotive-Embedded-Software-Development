/*
* DIO.h
*
* Created: 17-Aug-23 2:43:16 PM
*  Author: Salma
*/



#ifndef DIO_H_
#define DIO_H_

#include "ATMega32A_Registers.h"
#include "STD_types.h"
#include "Bit_math.h"

#define DIO_PORTA 0
#define DIO_PORTB 1
#define DIO_PORTC 2
#define DIO_PORTD 3

#define  DIO_PIN0 0
#define  DIO_PIN1 1
#define  DIO_PIN2 2
#define  DIO_PIN3 3
#define  DIO_PIN4 4
#define  DIO_PIN5 5
#define  DIO_PIN6 6
#define  DIO_PIN7 7

#define DIO_PIN_OUTPUT	1
#define DIO_PIN_INPUT	0

#define DIO_PORT_OUTPUT 0xFF
#define DIO_PORT_INPUT 0x00

#define DIO_PIN_HIGH	1
#define DIO_PIN_LOW		0

#define DIO_PORT_HIGH 0xFF
#define DIO_PORT_LOW  0x00

void DIO_SetPortDir (uint8_t Port , uint8_t Dir);
void DIO_SetPinDir (uint8_t Port , uint8_t Pin , uint8_t Dir);

void DIO_SetPortVal (uint8_t Port , uint8_t Val);
void DIO_SetPinVal (uint8_t Port , uint8_t Pin , uint8_t val);

void DIO_ReadPort (uint8_t Port , uint8_t* Val);
uint8_t DIO_ReadPin (uint8_t Port , uint8_t Pin );

void DIO_TogglePin (uint8_t Port , uint8_t Pin);

void DIO_SetPullUp(uint8_t Port , uint8_t Pin);


#endif /* DIO_H_ */