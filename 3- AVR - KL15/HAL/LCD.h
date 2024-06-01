/*
 * LCD.h
 *
 * Created: 18-Aug-23 10:13:11 AM
 *  Author: Salma
 */ 


#ifndef LCD_H_
#define LCD_H_

#include "LCD_Cfg.h"

#define LCD_WIDTH	20

#define MAX_NUMBER_OF_ROWS	4

typedef enum 
{
	LCD_no_error,
	LCD_MAX_SIZE_REACHED_ERROR,
	LCD_MAX_ROW_SIZE_EXCEEDED,
	LCD_MAX_COLUMN_SIZE_EXCEEDED
	}LCD_errors_status;

void LCD_Init(void);
void LCD_WriteCommand (uint8_t cmd);

void LCD_WriteChar (uint8_t chr);
LCD_errors_status LCD_WriteString(char* str);

LCD_errors_status LCD_Goto(uint8_t row, uint8_t col);
void LCD_Clear (void);

void LCD_WriteNumber( uint32_t num);

#endif /* LCD_H_ */