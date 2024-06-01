/*
 * LCD_Cfg.h
 *
 * Created: 18-Aug-23 10:22:36 AM
 *  Author: Salma
 */ 


#ifndef LCD_CFG_H_
#define LCD_CFG_H_

#include "DIO_interface.h"

#define LCD_MODE	4

 /*
    Sets interface data length (DL), number of display lines (N), and character font (F).
    DL = 1: 8 bits, DL = 0: 4 bits
    N = 1: 2 lines, N = 0: 1 line
    F = 1: 5 × 10 dots, F = 0: 5 × 8 dots
*/
#define FUNCTION_SET			   0x28
#define CLEAR_DISPLAY			   0x01
#define DISPLAY_ON_CURSOR_BLINKING 0x0F


#define LCD_4BIT_CMD_PORT	 DIO_PORTA  
#define LCD_4BIT_DATA_PORT	 DIO_PORTB  

#define  LCD_RS_PIN		DIO_PIN3
#define  LCD_RW_PIN     DIO_PIN1
#define  LCD_EN_PIN		DIO_PIN2
    
#define LCD_D4		DIO_PIN0
#define LCD_D5		DIO_PIN1
#define LCD_D6		DIO_PIN2
#define LCD_D7      DIO_PIN4
    


#endif /* LCD_CFG_H_ */