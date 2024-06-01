/*
 * LCD.h
 *
 * Created: 18-Aug-23 10:13:11 AM
 *  Author: Salma
 */ 


#ifndef LCD_H_
#define LCD_H_
/*******************************************************************************
 *                                Includes                                     *
 *******************************************************************************/
#include "LCD_Cfg.h"
#include "STD_TYPES.h"
/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
#define LCD_WIDTH	20
#define MAX_NUMBER_OF_ROWS	4
/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
typedef enum 
{
	LCD_no_error,
	LCD_MAX_SIZE_REACHED_ERROR,
	LCD_MAX_ROW_SIZE_EXCEEDED,
	LCD_MAX_COLUMN_SIZE_EXCEEDED
	}LCD_errors_status;
/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * @Fn			                        :   LCD_Init
 * @brief                               :   Initializes LCD with 4 bit mode , clears the display and blinks the cursor
 *
 * @param[in]   void                      
 *
 * @return      void                   
 *                                
 */
void LCD_Init(void);

/*
 * @Fn			                        :   LCD_WriteCommand
 * @brief                               :   Sends a command to the LCD
 *
 * @param[in]             cmd           :    LCD command
 *
 * @return                void                   
 *                                
 */
void LCD_WriteCommand (u8 cmd);

/*
 * @Fn			                        :   LCD_WriteChar
 * @brief                               :   Writes a character on the LCD
 *
 * @param[in]             chr           :   Character to write on LCD
 *
 * @return                void                   
 *                                
 */
void LCD_WriteChar (u8 chr);

/*
 * @Fn			                        :   LCD_WriteString
 * @brief                               :   Writes a string on the LCD
 *
 * @param[in]             str           :   String to write on LCD
 *
 * @return  LCD_MAX_SIZE_REACHED_ERROR  :   In case  the length of the string exceeds the lCD width
 *          LCD_no_error                :   In case of Successful Operation          
 */
LCD_errors_status LCD_WriteString(char* str);

/*
 * @Fn			                        :   LCD_Goto
 * @brief                               :   Goes to a certain row and column
 *
 * @param[in]             row           :   LCD row number
 *                        col           :   LCD column number
 *
 *  @return LCD_MAX_ROW_SIZE_EXCEEDED   :   In case the row number exceeds LCD number of rows
 *          LCD_MAX_COLUMN_SIZE_EXCEEDED:   In case the column number exceeds LCD number of columns
 *          LCD_no_error                :   In case of Successful Operation          
 */
LCD_errors_status LCD_Goto(u8 row, u8 col);

/*
 * @Fn			                        :   LCD_Clear
 * @brief                               :   Clears the LCD Display
 *
 * @param[in]   void                      
 *
 * @return      void                   
 *                                
 */
void LCD_Clear (void);

/*
 * @Fn			                        :   LCD_WriteNumber
 * @brief                               :   Writes a number on the LCD
 *
 * @param[in]             num           :   Number to write on LCD
 *
 * @return                void                   
 *                                
 */
void LCD_WriteNumber( u32 num);

#endif /* LCD_H_ */