/*
 * Vehicle_comm.c
 *
 * Created: 03-Sep-23 12:10:16 PM
 *  Author: Salma
 */
#include "STD_TYPES.h"

#include "LCD.h"
#include "LED_interface.h"
#include "Buzzer_interface.h"
#include "ADC.h"
#include "Keypad.h"

#include "Vehicle_comm.h"

#define F_CPU 16000000UL
#include <util/delay.h>

extern u16 Adc_Data;
static u8 Speed_limit = 50;
static u8 Gear_val = 'P';
extern u8 key;
u8 msg_recieved;

void APP_Init(void)
{
	SPI_MasterInit();
    LED_Init(LED_RED);
    BUZZER_Init(BUZZER1);
    LCD_Init();
    keyPad_Init();
    ADC_Init_N(VREF_VCC, ADC_SCALER_128);
}

void APP_DisplayMainMenu(void)
{
    LCD_Clear();

    LCD_Goto(0, 0);
    LCD_WriteString("     Main Menu      ");

    LCD_Goto(1, 0);
    LCD_WriteString("1-Runtime Values");

    LCD_Goto(2, 0);
    LCD_WriteString("2-Limit Speed");

    LCD_Goto(3, 0);
    LCD_WriteString("3-Set Speed Limit     ");
}

void APP_RuntimeValues(void)
{
    LCD_Clear();

    do
    {
        /*Display Current Gear*/
        LCD_Goto(2, 0);
        LCD_WriteString("Current Gear = ");
        LCD_Goto(2, 15);
        LCD_WriteChar(Gear_val);

        /* Update gear value if key is pressed */
        KeyPad_GetValue();
        if ((key == 'P') || (key == 'D') || (key == 'R') || (key == 'N'))
        {
            Gear_val = key;
            LCD_Goto(2, 15);
            LCD_WriteChar(Gear_val);
        }
        else
        {
            /*Do Nothing */
        }

        /*Display Current Speed*/
        LCD_Goto(0, 0);
        LCD_WriteString("Current Speed = ");
        ADC_ReadChannel(CHANNEL1);

        /*Check on Speed limit */
        switch (Gear_val)
        {
        case 'R':
            if (Adc_Data > MAX_SPEED_R_LIMIT)
            {
                LCD_Goto(0, 16);
                LCD_WriteString("   ");
                LCD_Goto(0, 16);
                LCD_WriteNumber(MAX_SPEED_R_LIMIT);
            }
            else
            {
                LCD_Goto(0, 16);
                LCD_WriteString("   ");
                LCD_Goto(0, 16);
                LCD_WriteNumber(Adc_Data);
                
                
                
            }
            break;

        case 'D':
            if (Adc_Data > Speed_limit)
            {
                LCD_Goto(0, 16);
                LCD_WriteString("   ");
                LCD_Goto(0, 16);
                LCD_WriteNumber(Speed_limit);
            }
            else
            {
                LCD_Goto(0, 16);
                LCD_WriteString("   ");
                LCD_Goto(0, 16);
                LCD_WriteNumber(Adc_Data);
            }
            break;

        case 'P':
            LCD_Goto(0, 16);
            LCD_WriteString("   ");
            LCD_Goto(0, 16);
            LCD_WriteNumber(MAX_SPEED_P_N_LIMIT);
            break;

        case 'N':
            LCD_Goto(0, 16);
            LCD_WriteString("   ");
            LCD_Goto(0, 16);
            LCD_WriteNumber(MAX_SPEED_P_N_LIMIT);
            break;

        default:
            break;

        }

    } while (key != '0');
}
void Check_limit(void)
{
    key = 'u';
    LCD_Clear();
    LCD_WriteString("Choose the Mode:");
    LCD_Goto(1, 0);
    LCD_WriteString("1: for OFF");
    LCD_Goto(2, 0);
    LCD_WriteString("2: for ON");
    LCD_Goto(3, 0);
    LCD_WriteString("0: to go back");
    do
    {
        KeyPad_GetValue();
    } while ((key != '1') && (key != '2') && (key != '0'));

    switch (key)
    {
    case '0':
        break;

    case '1':
        Speed_limit = MAX_SPEED_LIMIT;
        LCD_Clear();
        LCD_WriteString("Your speed limit is:");
        LCD_WriteNumber(Speed_limit);
        _delay_ms(3000);
        break;

    case '2':
        msg_recieved = SPI_Transceive(0);
         Speed_limit = msg_recieved;
        LCD_Clear();
        LCD_WriteString("Your speed limit is:");

        LCD_WriteNumber(Speed_limit);
        _delay_ms(3000);
        break;
    }
}
void Set_limit(void)
{
    LCD_Clear();
    LCD_WriteString("The new speed limit:");
    ADC_ReadChannel(CHANNEL1);
    LCD_Goto(2, 0);
      ADC_ReadChannel(CHANNEL1);
    LCD_WriteNumber(Adc_Data);
     SPI_Transceive(Adc_Data);
	 Speed_limit =Adc_Data;
    _delay_ms(2000);
}

void APP_runnable(void)
{
    key = 'u';
    APP_DisplayMainMenu();
    do
    {
        KeyPad_GetValue();
    } while ((key != '1') && (key != '2') && (key != '3'));

    switch (key)
    {
    case '1':
        APP_RuntimeValues();
        break;

    case '2':
        Check_limit();
        break;
        
    case '3':
        if (Gear_val == 'D')
        {
            Set_limit();
        }
        else
        {
            LCD_Clear();
            LCD_WriteString("Change gear to D !!");
            _delay_ms(2000);
        }

        break;
    default:
        break;
    }
}
