/*
 * ATMega32A_Registers.h
 *
 * Created: 18-Aug-23 9:30:17 AM
 *  Author: Ahmed hisham
 */ 


#ifndef ATMEGA32A_REGISTERS_H_
#define ATMEGA32A_REGISTERS_H_

#include "STD_types.h"


/***************** DIO Registers ************/
#define NULLPTR  ((void*)0x0)
#define NULL_PTR	NULLPTR

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


/********** ADC Registers ************/

#define ADMUX	*((volatile uint8_t*) 0x27)

#define MUX0	0
#define MUX1	1
#define MUX2	2
#define MUX3	3
#define MUX4	4
#define ADLAR	5
#define REFS0	6
#define REFS1	7

#define ADCSRA	*((volatile uint8_t*) 0x26)

#define ADPS0	0
#define ADPS1	1
#define ADPS2	2
#define ADIE	3
#define ADIF	4
#define ADATE	5
#define ADSC	6
#define ADEN	7

#define ADCH	*((volatile uint8_t*) 0x25)

#define ADCL	*((volatile uint8_t*) 0x24)

#define ADC_value	*((volatile uint16_t*) 0x24)

/********** Timers Registers ************/

#define TCCR0	*((volatile uint8_t*) 0x53)

#define CS00	0
#define CS01	1
#define CS02	2
#define WGM01	3
#define COM00	4
#define COM01	5
#define WGM00	6
#define FOC0	7


#define TCNT0	*((volatile uint8_t*) 0x52)

#define OCR0	*((volatile uint8_t*) 0x5C)

#define TIMSK	*((volatile uint8_t*) 0x59)

#define TOIE0	0
#define OCIE0	1
#define TOIE1	2
#define OCIE1B	3
#define OCIE1A	4
#define TICIE1	5
#define TOIE2	6
#define OCIE2	7

#define TIFR	*((volatile uint8_t*) 0x58)

#define TOV0	0
#define OCF0	1
#define TOV1	2
#define OCF1B	3
#define OCF1A	4
#define ICF1	5
#define TOV2	6
#define OCF2	7

#define SFIOR	*((volatile uint8_t*) 0x50)


#endif /* ATMEGA32A_REGISTERS_H_ */