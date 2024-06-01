/*
* DIO.c
*
* Created: 17-Aug-23 3:03:15 PM
*  Author: Salma
*/

#include "DIO.h"

void DIO_SetPortDir (uint8_t Port , uint8_t Dir)
{
	switch (Port)
	{
		case DIO_PORTA: DDRA = Dir;
		break;
		
		case DIO_PORTB: DDRB = Dir;
		break;
		
		case DIO_PORTC: DDRC = Dir;
		break;

		case DIO_PORTD: DDRD = Dir;
		break;
	}
}
void DIO_SetPinDir (uint8_t Port , uint8_t Pin , uint8_t Dir)
{
	switch (Dir)
	{
		case DIO_PIN_OUTPUT:
			switch (Port)
			{
				case DIO_PORTA: SET_BIT(DDRA,Pin);
				break;
				case DIO_PORTB: SET_BIT(DDRB,Pin);
				break;
				case DIO_PORTC: SET_BIT(DDRC,Pin);
				break;
				case DIO_PORTD: SET_BIT(DDRD,Pin);
				break;
			}
        break;
		
		case DIO_PIN_INPUT:
			switch (Port)
				{
					case DIO_PORTA: ClEAR_BIT(DDRA,Pin);
					break;
					case DIO_PORTB: ClEAR_BIT(DDRB,Pin);
					break;
					case DIO_PORTC: ClEAR_BIT(DDRC,Pin);
					break;
					case DIO_PORTD: ClEAR_BIT(DDRD,Pin);
					break;
		
				}
		break;
	}
	
}

void DIO_SetPortVal (uint8_t Port , uint8_t Val)
{
	switch (Port)
	{
		case DIO_PORTA: PORTA = Val;
		break;
		case DIO_PORTB: PORTB = Val;
		break;
		case DIO_PORTC: PORTC = Val;
		break;
		case DIO_PORTD: PORTD = Val;
		break;
	}
	
}
void DIO_SetPinVal (uint8_t Port , uint8_t Pin , uint8_t Val)
{ 

	switch (Val)
	{
		case DIO_PIN_HIGH:
		switch (Port)
		{
			case DIO_PORTA: SET_BIT(PORTA,Pin);
			break;
			case DIO_PORTB: SET_BIT(PORTB,Pin);
			break;
			case DIO_PORTC: SET_BIT(PORTC,Pin);
			break;
			case DIO_PORTD: SET_BIT(PORTD,Pin);
			break;
		}
		break;

		case DIO_PIN_LOW:
		switch (Port)
		{
			case DIO_PORTA: ClEAR_BIT(PORTA,Pin);
			break;
			case DIO_PORTB: ClEAR_BIT(PORTB,Pin);
			break;
			case DIO_PORTC: ClEAR_BIT(PORTC,Pin);
			break;
			case DIO_PORTD: ClEAR_BIT(PORTD,Pin);
			break;
		}
        break;
	}

	
}

void DIO_ReadPort (uint8_t Port , uint8_t* Val)
{
	switch (Port)
	{
		case DIO_PORTA: *Val = PINA;
		break;
		case DIO_PORTB: *Val = PINB;
		break;
		case DIO_PORTC: *Val = PINC;
		break;
		case DIO_PORTD: *Val = PIND;
		break;
	}
}
uint8_t DIO_ReadPin (uint8_t Port , uint8_t Pin )
{
    uint8_t Val=0;
	switch (Port)
	{
		case DIO_PORTA: Val = READ_BIT(PINA,Pin);
		break;
		case DIO_PORTB: Val = READ_BIT(PINB,Pin);
		break;
		case DIO_PORTC: Val = READ_BIT(PINC,Pin);
		break;
		case DIO_PORTD: Val = READ_BIT(PIND,Pin);
		break;
	}
    
    return Val;
	
}

void DIO_TogglePin (uint8_t Port , uint8_t Pin)
{
	switch (Port)
	{
		case DIO_PORTA: TOGGLE_BIT(PORTA,Pin);
		break;
		case DIO_PORTB: TOGGLE_BIT(PORTB,Pin);
		break;
		case DIO_PORTC: TOGGLE_BIT(PORTC,Pin);
		break;
		case DIO_PORTD: TOGGLE_BIT(PORTD,Pin);
		break;
	}
}

void DIO_SetPullUp(uint8_t Port , uint8_t Pin)
{
    SET_BIT(Port,Pin);
}