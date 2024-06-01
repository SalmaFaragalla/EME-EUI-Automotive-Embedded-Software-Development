/*
 * Keypad.c
 *
 * Created: 18-Aug-23 3:16:22 PM
 *  Author: Salma
 */

#include "Keypad_Cfg.h"
#define F_CPU 16000000UL
#include <util/delay.h>
#define  COL_INIT    7
#define  COL_FINAL   3

#define  ROW_INIT    5
#define  ROW_FINAL   2
const uint8_t KeyPad_Values[KEYPAD_COLUMNS_NUMBER][KEYPAD_COLUMNS_NUMBER] = 
{ 
    
{'1','2','3','4'},
{'5','6','7','8'},
{'9','A','B','C'},
{'D','E','F','G'} 
    
};

const uint8_t KeyPad_Cols [KEYPAD_COLUMNS_NUMBER] = {KEYPAD_COLUMN0,KEYPAD_COLUMN1,KEYPAD_COLUMN2,KEYPAD_COLUMN3};
const uint8_t KeyPad_Rows [KEYPAD_ROWS_NUMBER] = {KEYPAD_ROW0,KEYPAD_ROW1,KEYPAD_ROW2,KEYPAD_ROW3};
        
void keyPad_Init (void)
{
    	DIO_SetPinDir( KEYPAD_COLUMN_PORT , KEYPAD_COLUMN0 , DIO_PIN_INPUT);
    	DIO_SetPinDir( KEYPAD_COLUMN_PORT , KEYPAD_COLUMN1 , DIO_PIN_INPUT);
    	DIO_SetPinDir( KEYPAD_COLUMN_PORT , KEYPAD_COLUMN2 , DIO_PIN_INPUT);
    	DIO_SetPinDir( KEYPAD_COLUMN_PORT , KEYPAD_COLUMN3 , DIO_PIN_INPUT);
    	
    	DIO_SetPinDir( KEYPAD_ROW_PORT , KEYPAD_ROW0 , DIO_PIN_INPUT);
    	DIO_SetPinDir( KEYPAD_ROW_PORT , KEYPAD_ROW1 , DIO_PIN_INPUT);
    	DIO_SetPinDir( KEYPAD_ROW_PORT , KEYPAD_ROW2 , DIO_PIN_INPUT);
    	DIO_SetPinDir( KEYPAD_ROW_PORT , KEYPAD_ROW3 , DIO_PIN_INPUT);
        
        /*
	    DIO_SetPullUp( KEYPAD_ROW_PORT , KEYPAD_ROW0);
	    DIO_SetPullUp( KEYPAD_ROW_PORT , KEYPAD_ROW1);
	    DIO_SetPullUp( KEYPAD_ROW_PORT , KEYPAD_ROW2);
	    DIO_SetPullUp( KEYPAD_ROW_PORT , KEYPAD_ROW3);        
        */
        
        /*
        DIO_SetPinVal(KEYPAD_COLUMN_PORT , KEYPAD_COLUMN0 , DIO_PIN_LOW);
        DIO_SetPinVal(KEYPAD_COLUMN_PORT , KEYPAD_COLUMN1 , DIO_PIN_LOW);
        DIO_SetPinVal(KEYPAD_COLUMN_PORT , KEYPAD_COLUMN2 , DIO_PIN_LOW);
        DIO_SetPinVal(KEYPAD_COLUMN_PORT , KEYPAD_COLUMN3 , DIO_PIN_LOW);
*/

}

uint8_t KeyPad_GetValue(void)
{ 
    uint8_t cols = 0;
    uint8_t rows = 0;

   
    for (rows = 0; rows < KEYPAD_ROWS_NUMBER ; ++rows)
    {
        DIO_SetPinDir( KEYPAD_COLUMN_PORT ,KeyPad_Rows[rows] , DIO_PIN_OUTPUT);
        DIO_SetPinVal(KEYPAD_COLUMN_PORT ,KeyPad_Rows[rows], DIO_PIN_LOW);
        
        for (cols= 0; cols< KEYPAD_ROWS_NUMBER ; ++cols)
        {
             
            if (DIO_ReadPin(  KEYPAD_COLUMN_PORT , KeyPad_Cols[cols]) == DIO_PIN_LOW)
            {
                _delay_ms(100);
 
                 if ( DIO_ReadPin(  KEYPAD_COLUMN_PORT , KeyPad_Cols[cols]) == DIO_PIN_LOW)
                 {
                     DIO_SetPinDir( KEYPAD_COLUMN_PORT ,KeyPad_Rows[rows] , DIO_PIN_INPUT);
                     
                        return KeyPad_Values[rows][cols];                          
                 };

            }
            
        }
        
          DIO_SetPinDir( KEYPAD_COLUMN_PORT ,KeyPad_Rows[rows] , DIO_PIN_INPUT);
    }
    
  return KeyPad_Values[rows][cols];
  
  


/*
uint8_t Col_Loc = 0;
uint8_t Row_Loc = 0;

uint8_t Value = 0;
uint8_t Temp = 0;

for(Col_Loc = COL_INIT ; Col_Loc >= COL_FINAL ; Col_Loc--)
{
    //Activate the column
    if (Col_Loc == 4)
    {
        continue;
    }
    DIO_SetPinVal(KEYPAD_COLUMN_PORT , Col_Loc , DIO_PIN_LOW);
    
    for(Row_Loc = ROW_INIT ; Row_Loc >= ROW_FINAL ; Row_Loc--)
    {
        
        Temp = DIO_ReadPin(KEYPAD_ROW_PORT , Row_Loc);
        
        //wait until the switch is releases
        if(!Temp)
        {
            Value =  KeyPad_Values[Row_Loc - ROW_INIT][Col_Loc - COL_INIT];
            
            while(!Temp)
            {
                Temp = DIO_ReadPin(KEYPAD_ROW_PORT , Row_Loc);
            }
            
            //Delay to avoid bouncing , bouncing occurs in mechanical switches and can cause one switch press to be detected as multiple presses
            _delay_ms(10);
        }
        
    }
    //deactivate column again
    DIO_SetPinVal(KEYPAD_COLUMN_PORT , Col_Loc , DIO_PIN_HIGH);
    
}

return Value ;

*/
}