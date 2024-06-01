/*
    * sunrise_alarm.c
    *
    * Created: 8/28/2023 3:04:57 PM
    *  Author:nada
    */

#include "sunrise_alarm.h"
#include <avr/interrupt.h>
#include <stdlib.h>

    extern u16 Adc_Data;
    extern u8 key;
    alarm INPUT_TIMERS[3]={{-1,-1,-1,-1},{-1,-1,-1,-1},{-1,-1,-1,-1}};
    u8 last_timer;

   volatile u16 seconds=0;
   volatile u16 minutes=0;
   volatile u16 ticks;
    u8 clear_flag=1;
    match_status matching=NO_MATCH;
	
	st_timer0_config_t_ timer0_config;

    void sorting_timers(alarm* INPUT_TIMERS)
    {
        alarm temp;

        for(int k=0; k<last_timer; k++)
        {
            for(int l=0; l<last_timer-k; l++)
            {
                if(INPUT_TIMERS[l].minutes_relative<INPUT_TIMERS[l+1].minutes_relative)
                {
                    temp= INPUT_TIMERS[l];
                    INPUT_TIMERS[l]=INPUT_TIMERS[l+1];
                    INPUT_TIMERS[l+1]=temp;
                }
                else if(INPUT_TIMERS[l].minutes_relative==INPUT_TIMERS[l+1].minutes_relative)
                {
                    if(INPUT_TIMERS[l].seconds_relative<INPUT_TIMERS[l+1].seconds_relative)
                    {
                        temp= INPUT_TIMERS[l];
                        INPUT_TIMERS[l]=INPUT_TIMERS[l+1];
                        INPUT_TIMERS[l+1]=temp;
                    }
                }
            }
        }
    }

    error_status SET(void)
    {
        uint8_t array_section[2]= {0};
        char input_string[6]= {0};

        input_string[5]='\0';
        if(last_timer>=3)
        {
            LCD_Clear();
            LCD_WriteString("timers full");
            return ERROR;
        }
        uint8_t i=0;
        LCD_Goto(0,0);
        LCD_WriteString("Enter alarm time :");
        LCD_Goto(1,7);
        LCD_WriteString("00:00");
        LCD_Goto(1,7);
        char Pressed_Key=key;

        while(Pressed_Key != '=')
        {
            
            while(KeyPad_GetValue()!=PRESSED);

			Pressed_Key=key;
            input_string[i]=Pressed_Key;

            LCD_WriteChar(Pressed_Key);
            i++;
            if(i==2)
                LCD_Goto(1,10);
            if(i>4)
            {
                i=0;
                LCD_Goto(1,7);
            }
        }
        array_section[0]=input_string[0];
        array_section[1]=input_string[1];
        if(atoi(array_section)>=0 && atoi(array_section)<60)
            INPUT_TIMERS[last_timer].minutes_user=atoi(array_section);
        else
        {
            LCD_Goto(3,0);
            LCD_WriteString("invalid");
            return ERROR;
        }
        array_section[0]=input_string[2];
        array_section[1]=input_string[3];

        if(atoi(array_section)>=0 && atoi(array_section)<60)
        {
            INPUT_TIMERS[last_timer].seconds_user=atoi(array_section);
        }
        else
        {
            LCD_Goto(3,0);
            LCD_WriteString("invalid");
            return ERROR;
        }
        INPUT_TIMERS[last_timer].minutes_relative= minutes + INPUT_TIMERS[last_timer].minutes_user;
        INPUT_TIMERS[last_timer].seconds_relative= seconds + INPUT_TIMERS[last_timer].seconds_user;
		if(INPUT_TIMERS[last_timer].seconds_relative >60)
			{
				INPUT_TIMERS[last_timer].seconds_relative-=60;
				INPUT_TIMERS[last_timer].minutes_relative++;
			}
        last_timer++;
        LCD_Goto(3,0);
        LCD_WriteString("success");
        sorting_timers(INPUT_TIMERS);

        return NO_ERROR;
    }


    error_status cancel_timer(uint8_t Timer_num)
    {
        if(Timer_num>3||Timer_num<=0)
        {
            return Timer_Not_found;
        }
        else
        {
            INPUT_TIMERS[Timer_num-1].minutes_user=-1;
            INPUT_TIMERS[Timer_num-1].seconds_user=-1;
            INPUT_TIMERS[Timer_num-1].seconds_relative=-1;
            INPUT_TIMERS[Timer_num-1].minutes_relative=-1;
            sorting_timers(INPUT_TIMERS);

            last_timer--;
            return Time_deleted_successfully;

        }
    }



    u8 LDR_check(void)
    {

        ADC_ReadChannel(CHANNEL0);
        LCD_Goto(3,0);
        LCD_WriteNumber(Adc_Data);
        if (Adc_Data > 50)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    error_status Waking_sequence_Init(void)
    {
        
        u8 buzzer_time_sec = 7;
        u8 ldr_time_sec = 7;

        LCD_Clear();
        LCD_Goto(0, 0);
        LCD_WriteString("     ALARM TIME     ");

        /*LED Brightening */
        LCD_Goto(2, 0);
        LCD_WriteString("Light brightening...");
        /*Configuring the timer*/
        

        for (u8 i = 2; i < 253; ++i)
        {
            if (!LDR_check())
            {
                TIMER0_set_compare(i);

            }
            else
            {
                LCD_Clear();
                LCD_WriteString("    Good Morning     ");

                return;
            }
        }


        LCD_Goto(2, 0);
        LCD_WriteString("                    ");
        LCD_Goto(2, 0);
        LCD_WriteString("Buzzer will turn on in ");

        /*Check exact name */
        buzzer_time_sec += seconds;
        while (buzzer_time_sec < seconds)
        {
            LCD_WriteNumber(buzzer_time_sec-seconds);
            if (LDR_check())
            {
                LCD_Clear();
                LCD_WriteString("    Good Morning     ");

                return;
            }
        }
        LCD_Goto(2, 0);
        LCD_WriteString("                    ");
        LCD_Goto(2, 0);
        LCD_WriteString("      Buzzer On     ");
        BUZZER_TurnOn(BUZZER1);

        while (!LDR_check());
        ldr_time_sec+=seconds;
        while (ldr_time_sec<seconds);


        BUZZER_TurnOff(BUZZER1);
        LCD_Clear();
        LCD_WriteString("    Good Morning     ");
        matching==NO_MATCH;

        return;
    }

    void MATCH(void)
    {
        u8 minutes_matchflag=0;
        u8 seconds_matchflag=0;
        u8 index_matching;

        for (u8 array =0; array < 3 ; array++)
        {
            if (INPUT_TIMERS[array].minutes_relative==minutes)
            {
                minutes_matchflag =1;
                if (INPUT_TIMERS[array].seconds_relative==seconds)
                {
                    seconds_matchflag=1;
					break;
                }
                else
                {
                    seconds_matchflag=0;
                }
            }
            else
            {
                minutes_matchflag=0;
            }
        }

        if (minutes_matchflag ==1 && seconds_matchflag ==1 )
        {
            cancel_timer(index_matching+1);
            matching = MATCHED;
        }
        else
        {
            matching = NO_MATCH;
        }
    }

    void count_time()
    {
        ticks++;
        if (ticks>61)
        {
            seconds++;
            TCNT2=246;
            ticks =0;
        }
        if (seconds>59)
        {
            minutes++;
            seconds=0;
        }
    }

    void Display_menu(void)
    {
        LCD_Clear();
        LCD_Goto (0,0);
        LCD_WriteString("   Sunrise Alarm    ");
        LCD_Goto(1,0);
        LCD_WriteString(" + : Set Alarm");
        LCD_Goto(2,0);
        LCD_WriteString(" - : Cancel Alarm");
        LCD_Goto(3,0);
        LCD_WriteString(" * : Show Alarms ");
        LCD_Goto(3,19);
    }

    error_status Timers_Display(void)
    {
        LCD_Clear();
        LCD_WriteString("The running timers:");
        char buffer[2]= {0};
        for(int i=0; i<last_timer; i++)
        {
            LCD_Goto(i+1,0);
            LCD_WriteNumber(i+1);
            LCD_WriteChar('-');
            LCD_Goto(i+1,3);
            LCD_WriteNumber(INPUT_TIMERS[i].minutes_relative);
            LCD_WriteChar(':');
            LCD_WriteNumber(INPUT_TIMERS[i].seconds_relative);

        }
        LCD_Goto(3,14);
        LCD_WriteString("0:Back");
        clear_flag=0;
    }


    void runnable(void)
    {
        if(clear_flag ==1)
        {
            Display_menu(); 
            do
            {
                MATCH();
                if (matching==MATCHED)
                {
                    Waking_sequence_Init();
                }

               KeyPad_GetValue();
                

            }
            while((key!='+' && key!='*' && key!='-' ));
        }

        LCD_Clear();

        if (key  =='0')
        {
            clear_flag =1;
        }
        switch (key)
        {
        case '+':
            SET();
            MATCH();
            if (matching==MATCHED)
            {
                Waking_sequence_Init();
            }
            break;

        case '-':
            key='u';
            do
            {
                KeyPad_GetValue();
            }
            while(key=='u');
			if(key=='1')
				cancel_timer(1);
			else if (key=='2')
				cancel_timer(2);
			else if(key=='3')
				cancel_timer(3);
            key='u';
            MATCH();
            if (matching==MATCHED)
            {
                Waking_sequence_Init();
            }
            break;

        case'*':
            Timers_Display();
            while(key !='0')
            {
                KeyPad_GetValue();
                MATCH();
                if (matching==MATCHED)
                {
                    Waking_sequence_Init();
                    break;
                }
                
            }


        
        }

        MATCH();
        if (matching==MATCHED)
        {
            Waking_sequence_Init();
        }
    }
	
void Init(){
	LCD_Init();
	DIO_SetPinDirection(DIO_PORTB,DIO_PIN3,DIO_PIN_OUTPUT);
	TIMER_ConfigType ConfigStruct = {NORMAL_MODE,F_CPU_1024_2,247};
	TIMER2_Init(&ConfigStruct);
	keyPad_Init();
	ADC_Init_N(VREF_VCC, ADC_SCALER_128);
	timer0_config.enu_timer0_prescaler = TIMER_PRESCALER_1024;
	timer0_config.enu_timer0_mode = PWM_FAST;
	timer0_config.enu_timer0_int_en = POLLING;
	timer0_config.enu_timer0_oc0 = CLEAR_OC0;
	timer0_config.compare_val = 0;
	timer0_config.preload = 0;
	TIMER0_Init(&timer0_config);
	TIMER2_setCallBack(count_time);
	SET_BIT(SREG,7);
}