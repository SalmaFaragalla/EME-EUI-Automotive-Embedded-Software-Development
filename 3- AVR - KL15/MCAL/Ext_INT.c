/*
 * Ext_INT.c
 *
 * Created: 8/19/2023 10:13:54 AM
 *  Author: eme
 */ 

#include "Bit_math.h"
#include "ATMega32A_Registers.h"
#include "Ext_INT.h"
#include "Button.h"
#include "LED.h"


static void (*INT0_Fptr) (void)=NULLPTR;
static void (*INT1_Fptr) (void)=NULLPTR;
static void (*INT2_Fptr) (void)=NULLPTR;

void EXI_SetCallBack(void(*LocalPtr)(void))
{
	#if INterrupt_ON ==EXTENAL_INT0 
      	INT0_Fptr=LocalPtr;
    #elif INterrupt_ON ==EXTENAL_INT1 
      	INT1_Fptr=LocalPtr;
    #elif INterrupt_ON ==EXTENAL_INT2 
	  	INT2_Fptr=LocalPtr;

	#endif
}
void SET_GlobalInterrupt (void)
{
	#if GLOBAL_INTERRUPT == GLOBAL_INTERRUPT_ENABLE
	
	SET_BIT(SREG,7);
	
	#elif GLOBAL_INTERRUPT == GLOBAL_INTERRUPT_DISABLE
	ClEAR_BIT(SREG,7);
	
	#endif
	
}

void EXTERNAL_INT0_Init (void)
{
	/* MCUCR – MCU Control Register
	The MCU Control Register contains control bits for interrupt sense control and general MCU functions*/
	/* Bit 1, 0 – ISC01, ISC00: Interrupt Sense Control 0 Bit 1 and Bit 0*/
	/*Table 14-1. Interrupt 1 Sense Control*/
			
		SET_GlobalInterrupt();
		SET_BIT(GICR,EXTENAL_INT0);
		
		#if EXTENAL_INT0_TRIGGER == INT0_TRIGGER_LOW_LEVEL
		
		ClEAR_BIT(MCUCR,ISC00);
		ClEAR_BIT(MCUCR,ISC01);
		
		#elif EXTENAL_INT0_TRIGGER == INT0_TRIGGER_ANY_LOGICAL_CHANGE
		
		SET_BIT(MCUCR,ISC00);
		ClEAR_BIT(MCUCR,ISC01);
		
		#elif EXTENAL_INT0_TRIGGER == INT0_TRIGGER_FALLING_EDGE
		
		SET_BIT(MCUCR,ISC01);
		ClEAR_BIT(MCUCR,ISC00);
		
		#elif EXTENAL_INT0_TRIGGER == INT0_TRIGGER_RISING_EDGE
		
		SET_BIT(MCUCR,ISC00);
		SET_BIT(MCUCR,ISC01);
		
		#endif
	
	
}
void EXTERNAL_INT1_Init (void)
{
	/* MCUCR – MCU Control Register
	The MCU Control Register contains control bits for interrupt sense control and general MCU functions*/
	/* Bit 3, 2 – ISC11, ISC10: Interrupt Sense Control 1 Bit 1 and Bit 0 */
	/*Table 14-2. Interrupt 0 Sense Control*/
	
	
		SET_GlobalInterrupt();
		SET_BIT(GICR,EXTENAL_INT1);
		
		#if EXTENAL_INT1_TRIGGER == INT1_TRIGGER_LOW_LEVEL
			
		ClEAR_BIT(MCUCR,ISC10);
		ClEAR_BIT(MCUCR,ISC11);
			
		#elif EXTENAL_INT1_TRIGGER == INT1_TRIGGER_ANY_LOGICAL_CHANGE
			
		SET_BIT(MCUCR,ISC10);
		ClEAR_BIT(MCUCR,ISC11);
			
		#elif EXTENAL_INT1_TRIGGER == INT1_TRIGGER_FALLING_EDGE
			
		SET_BIT(MCUCR,ISC11);
		ClEAR_BIT(MCUCR,ISC10);
			
		#elif EXTENAL_INT1_TRIGGER == INT1_TRIGGER_RISING_EDGE
			
		SET_BIT(MCUCR,ISC10);
		SET_BIT(MCUCR,ISC11);
			
		#endif
        
   
}
void EXTERNAL_INT2_Init (void)
{
	/*2 MCUCSR – MCU Control and Status Register*/
	/* Bit 6 – ISC2: Interrupt Sense Control 2*/
	
		SET_GlobalInterrupt();
		SET_BIT(GICR,EXTENAL_INT2);
		
		
		#if EXTENAL_INT2_TRIGGER == INT2_TRIGGER_FALLING_EDGE
		
		ClEAR_BIT(MCUCR,ISC2);
		
		#elif EXTENAL_INT2_TRIGGER == INT2_TRIGGER_RISING_EDGE
		
		SET_BIT(MCUCR,ISC2);
		
		#endif
		
}
void EXTERNAL_INT0_Disable (void){
    ClEAR_BIT(GICR,EXTENAL_INT0);
}

void EXTERNAL_INT1_Disable (void){
     ClEAR_BIT(GICR,EXTENAL_INT1);
}    
void EXTERNAL_INT2_Disable (void){
     ClEAR_BIT(GICR,EXTENAL_INT2);
}

 ISR(INT1_vect){
   if (INT1_Fptr!=NULLPTR)
   {
	   INT1_Fptr();
   }
  
  
}
