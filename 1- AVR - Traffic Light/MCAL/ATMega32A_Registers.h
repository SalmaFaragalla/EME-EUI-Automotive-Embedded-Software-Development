/*
 * ATMega32A_Registers.h
 *
 * Created: 18-Aug-23 9:30:17 AM
 *  Author: Salma
 */ 


#ifndef ATMEGA32A_REGISTERS_H_
#define ATMEGA32A_REGISTERS_H_

#include "STD_types.h"


/***************** DIO Registers ************/
#define NULLPTR  ((void*)0x0)
#define PORTA	*((volatile uint8_t*) 0x3B)
#define DDRA	*((volatile uint8_t*) 0x3A)
#define PINA	*((volatile uint8_t*) 0x39)

#define PORTB	*((volatile uint8_t*) 0x38)
#define DDRB	*((volatile uint8_t*) 0x37)
#define PINB	*((volatile uint8_t*) 0x36)

#define PORTC	*((volatile uint8_t*) 0x35)
#define DDRC	*((volatile uint8_t*) 0x34)
#define PINC	*((volatile uint8_t*) 0x33)

#define PORTD	*((volatile uint8_t*) 0x32)
#define DDRD	*((volatile uint8_t*) 0x31)
#define PIND	*((volatile uint8_t*) 0x30)

/********** Interrupt Registers ************/

#define MCUCR	*((volatile uint8_t*) 0x55)
#define GICR	*((volatile uint8_t*) 0x5B)
#define MCUCSR	*((volatile uint8_t*) 0x54)
#define SREG	*((volatile uint8_t*) 0x5F)

#endif /* ATMEGA32A_REGISTERS_H_ */