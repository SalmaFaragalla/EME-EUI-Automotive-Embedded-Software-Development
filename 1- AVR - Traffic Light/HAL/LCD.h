/*
 * LCD.h
 *
 * Created: 18-Aug-23 10:13:11 AM
 *  Author: Salma
 */ 


#ifndef LCD_H_
#define LCD_H_

#include "LCD_Cfg.h"

void LCD_Init(void);
void LCD_WriteCommand (uint8_t cmd);

void LCD_WriteChar (uint8_t chr);
void LCD_WriteString(char* str);

void LCD_Goto(uint8_t row, uint8_t col);
void LCD_Clear (void);

#endif /* LCD_H_ */