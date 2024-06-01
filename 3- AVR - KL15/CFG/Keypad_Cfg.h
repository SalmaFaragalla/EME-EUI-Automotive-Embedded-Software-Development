/*
 * Keypad_Cfg.h
 *
 * Created: 18-Aug-23 3:00:00 PM
 *  Author: Salma
 */ 


#ifndef KEYPAD_CFG_H_
#define KEYPAD_CFG_H_

#include "DIO.h"

#define KEYPAD_COLUMNS_NUMBER 4
#define KEYPAD_ROWS_NUMBER 4

#define  KEYPAD_COLUMN_PORT  DIO_PORTD
#define  KEYPAD_ROW_PORT     DIO_PORTC

#define  KEYPAD_COLUMN0      DIO_PIN7
#define  KEYPAD_COLUMN1      DIO_PIN6
#define  KEYPAD_COLUMN2      DIO_PIN5
#define  KEYPAD_COLUMN3      DIO_PIN3

#define  KEYPAD_ROW0        DIO_PIN5
#define  KEYPAD_ROW1        DIO_PIN4
#define  KEYPAD_ROW2        DIO_PIN3
#define  KEYPAD_ROW3        DIO_PIN2

#endif /* KEYPAD_CFG_H_ */