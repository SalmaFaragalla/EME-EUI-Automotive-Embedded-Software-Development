/*
 * LCD.c
 *
 * Created: 18-Aug-23 10:32:12 AM
 *  Author: Salma
 */ 
#include "LCD.h"
#include "Bit_math.h"
#define F_CPU 16000000UL
#include <util/delay.h>

/*HDD44780 datasheet*/
/*https://www.8051projects.net/lcd-interfacing/commands.php*/

void LCD_Init(void)
{
    
    DIO_SetPinDir(LCD_4BIT_CMD_PORT ,LCD_RS_PIN ,DIO_PIN_OUTPUT);
    /*DIO_SetPinDir(LCD_4BIT_CMD_PORT ,LCD_RW_PIN ,DIO_PIN_OUTPUT);*/
    DIO_SetPinDir(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_OUTPUT);
    
    DIO_SetPinDir(LCD_4BIT_DATA_PORT ,LCD_D4 ,DIO_PIN_OUTPUT);
    DIO_SetPinDir(LCD_4BIT_DATA_PORT ,LCD_D5 ,DIO_PIN_OUTPUT);
    DIO_SetPinDir(LCD_4BIT_DATA_PORT ,LCD_D6 ,DIO_PIN_OUTPUT);
    DIO_SetPinDir(LCD_4BIT_DATA_PORT ,LCD_D7 ,DIO_PIN_OUTPUT);
    
   _delay_ms(1);


    LCD_WriteCommand (FUNCTION_SET);
    _delay_ms(1);
    LCD_WriteCommand (CLEAR_DISPLAY); 
    _delay_ms(1);
    LCD_WriteCommand (DISPLAY_ON_CURSOR_BLINKING); 
    _delay_ms(1);

    
    
}

void LCD_WriteCommand(uint8_t cmd)
{
    /*
    RS (Register Select)
    RS = 0 -> Command Register is selected
    */
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_RS_PIN ,DIO_PIN_LOW);
    _delay_ms(1);

    /*
    For 4-bit interface data, only four bus lines (DB4 to DB7) are used for transfer. Bus lines DB0 to DB3
    are disabled. The data transfer between the HD44780U and the MPU is completed after the 4-bit data
    has been transferred twice. As for the order of data transfer, the four high order bits (for 8-bit operation,
    DB4 to DB7) are transferred before the four low order bits (for 8-bit operation, DB0 to DB3).
    */
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D4 , READ_BIT(cmd,4));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D5 , READ_BIT(cmd,5));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D6 , READ_BIT(cmd,6));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D7 , READ_BIT(cmd,7));
    _delay_ms(1);
    
    
    /*
    Falling Edge to process the data
    */
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_HIGH);
    _delay_ms(1);
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_LOW);      
    _delay_ms(1);


    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D4 , READ_BIT(cmd,0));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D5 , READ_BIT(cmd,1));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D6 , READ_BIT(cmd,2));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D7 , READ_BIT(cmd,3));
    _delay_ms(1);
    
    /*
    Falling Edge to process the data
    */
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_HIGH);     
    _delay_ms(1);
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_LOW);
    
    _delay_ms(5); /*LCD store delay*/
}

void LCD_WriteChar(uint8_t chr)
{  
    /*
    RS (Register Select)
    RS = 1 -> Data Register is selected
    */
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_RS_PIN ,DIO_PIN_HIGH);
    _delay_ms(1);

    
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D4 , READ_BIT(chr,4));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D5 , READ_BIT(chr,5));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D6 , READ_BIT(chr,6));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D7 , READ_BIT(chr,7));
    _delay_ms(1);
    
    
    /*
    Falling Edge to process the data
    */
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_HIGH);
    _delay_ms(1);
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_LOW);     
    _delay_ms(1);


    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D4 , READ_BIT(chr,0));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D5 , READ_BIT(chr,1));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D6 , READ_BIT(chr,2));
    DIO_SetPinVal (LCD_4BIT_DATA_PORT, LCD_D7 , READ_BIT(chr,3));
    _delay_ms(1);
    
    /*
    Falling Edge to process the data
    */
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_HIGH);     /*Enable=1*/
    _delay_ms(1);
    DIO_SetPinVal(LCD_4BIT_CMD_PORT ,LCD_EN_PIN ,DIO_PIN_LOW);
    
    _delay_ms(5); /* LCD store delay*/
}

LCD_errors_status LCD_WriteString(char* str)
{
	
    uint8_t i = 0;
    
    while(str[i] != '\0')
    {
        LCD_WriteChar(str[i]);
        i++;
		if(i>LCD_WIDTH)
			return LCD_MAX_SIZE_REACHED_ERROR;
    }
	return LCD_no_error;
}


LCD_errors_status LCD_Goto(uint8_t row, uint8_t col)
{
	
	if(row>MAX_NUMBER_OF_ROWS)
		return LCD_MAX_ROW_SIZE_EXCEEDED;
		
	else if (col>LCD_WIDTH)
		return LCD_MAX_COLUMN_SIZE_EXCEEDED;
	
	uint8_t pos[] = { 0x80, 0xC0, 0xA0, 0xE0};
	
	LCD_WriteCommand(pos[row]+col);
	
	return LCD_no_error;

}

void LCD_Clear(void)
{
    LCD_WriteCommand(0x01);
}

void LCD_WriteNumber( uint32_t num)
{
	uint8_t str[10],i=0,j;
	if (num==0)
	{
		LCD_WriteChar('0');
		return;
	}
	if (num<0)
	{
		num=num*(-1);
		LCD_WriteChar('-');
	}
	
	while(num)
	{
		str[i]=num%10 +'0';
		i++;
		num=num/10;
	}
	for (j=i;j>0;j--)
	{
		LCD_WriteChar(str[j-1]);
	}

}
