/*
 * Keypad.c
 *
 * Created: 18-Aug-23 3:16:22 PM

 */

#include "Keypad.h"

u8 kepad_value;
#define  COL_INIT    7
#define  COL_FINAL   3

#define  ROW_INIT    5
#define  ROW_FINAL   2

 u8 key = 'u';

 const u8 KeyPad_Values[KEYPAD_COLUMNS_NUMBER][KEYPAD_COLUMNS_NUMBER] =
 {

	 {'1', '2', '3', 'P'},
	 {'4', '5', '6', 'R'},
	 {'7', '8', '9', 'N'},
	 {'@', '0', '#', 'D'}

 };

 const u8 KeyPad_Cols[KEYPAD_COLUMNS_NUMBER] = {KEYPAD_COLUMN0, KEYPAD_COLUMN1, KEYPAD_COLUMN2, KEYPAD_COLUMN3};
 const u8 KeyPad_Rows[KEYPAD_ROWS_NUMBER] = {KEYPAD_ROW0, KEYPAD_ROW1, KEYPAD_ROW2, KEYPAD_ROW3};

 void keyPad_Init(void)
 {
	 DIO_SetPinDirection(KEYPAD_COLUMN_PORT, KEYPAD_COLUMN0, DIO_PIN_INPUT);
	 DIO_SetPinDirection(KEYPAD_COLUMN_PORT, KEYPAD_COLUMN1, DIO_PIN_INPUT);
	 DIO_SetPinDirection(KEYPAD_COLUMN_PORT, KEYPAD_COLUMN2, DIO_PIN_INPUT);
	 DIO_SetPinDirection(KEYPAD_COLUMN_PORT, KEYPAD_COLUMN3, DIO_PIN_INPUT);

	 DIO_SetPinDirection(KEYPAD_ROW_PORT, KEYPAD_ROW0, DIO_PIN_INPUT);
	 DIO_SetPinDirection(KEYPAD_ROW_PORT, KEYPAD_ROW1, DIO_PIN_INPUT);
	 DIO_SetPinDirection(KEYPAD_ROW_PORT, KEYPAD_ROW2, DIO_PIN_INPUT);
	 DIO_SetPinDirection(KEYPAD_ROW_PORT, KEYPAD_ROW3, DIO_PIN_INPUT);

	 DIO_SetPullUp(KEYPAD_ROW_PORT, KEYPAD_ROW0);
	 DIO_SetPullUp(KEYPAD_ROW_PORT, KEYPAD_ROW1);
	 DIO_SetPullUp(KEYPAD_ROW_PORT, KEYPAD_ROW2);
	 DIO_SetPullUp(KEYPAD_ROW_PORT, KEYPAD_ROW3);
 }

KEYPAD_ERROR_T  KeyPad_GetValue(void)
{
	u8 cols = 0;
	u8 rows = 0;
	KEYPAD_ERROR_T status=NOT_PRESSED ;
	u8 Pin_value;

	for (rows = 0; rows < KEYPAD_ROWS_NUMBER; ++rows)
	{
		DIO_SetPinDirection(KEYPAD_ROW_PORT, KeyPad_Rows[rows], DIO_PIN_OUTPUT);
		DIO_SetPinValue(KEYPAD_ROW_PORT, KeyPad_Rows[rows], DIO_PIN_LOW);

		for (cols = 0; cols < KEYPAD_COLUMNS_NUMBER; ++cols)
		{
			DIO_GetPinValue(KEYPAD_COLUMN_PORT, KeyPad_Cols[cols],&Pin_value);
			if (Pin_value == DIO_PIN_LOW)
			{
				while (Pin_value == DIO_PIN_LOW){
					DIO_GetPinValue(KEYPAD_COLUMN_PORT, KeyPad_Cols[cols],&Pin_value);
				}
				DIO_SetPinDirection(KEYPAD_ROW_PORT, KeyPad_Rows[rows], DIO_PIN_INPUT);
				key= KeyPad_Values[rows][cols];
				status =PRESSED;
				return status;
				
			}
		}

		DIO_SetPinDirection(KEYPAD_ROW_PORT, KeyPad_Rows[rows], DIO_PIN_INPUT);
	}

	return status;
}
