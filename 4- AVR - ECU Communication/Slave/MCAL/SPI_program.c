/*
 * SPI_program.c
 *
 * Created: 01-Sep-23 1:01:06 PM
 *  Author: Salma
 */

#include "BIT_MATH.h"

#include "GIE_Interface.h"

#include "SPI_config.h"
#include "SPI_private.h"
#include "SPI_interface.h"
#include <avr/interrupt.h>

#define F_CPU 16000000UL
#include <util/delay.h>

static void (*SPI_g_Callback)(void);

void SPI_Init()
{
    u8 Local_u8SPCR = 0;

    /* SPI Interrupt */
#if SPI_INT_STATUS == SPI_INT_ENABLE
    SET_BIT(Local_u8SPCR, SPCR_SPIE);
    GIE_Enable();
#elif SPI_INT_STATUS == SPI_INT_DISABLE
    CLR_BIT(Local_u8SPCR, SPCR_SPIE);
#else
#error "Wrong SPI_INT_STATUS config "
#endif

    /* Data Order */
#if SPI_DATA_ORDER == SPI_DATA_LSB_FIRST
    SET_BIT(Local_u8SPCR, SPCR_DORD);
#elif SPI_DATA_ORDER == SPI_DATA_MSB_FIRST
    CLR_BIT(Local_u8SPCR, SPCR_DORD);
#else
#error "WrongSPI_DATA_ORDER config "
#endif

    /* Clock Polarity */
#if SPI_CLK_POLARITY == SPI_RISING_LEADING_FALLING_TRAILING
    CLR_BIT(Local_u8SPCR, SPCR_CPOL);
#elif SPI_CLK_POLARITY == SPI_FALLING_LEADING_RISING_TRAILING
    SET_BIT(Local_u8SPCR, SPCR_CPOL);
#else
#error "Wrong SPI_CLK_POLARITY config"
#endif

    /* Clock Phase*/
#if SPI_CLK_PHASE == SPI_SAMPLE_LEADING_SETUP_TRAILING
    CLR_BIT(Local_u8SPCR, SPCR_CPHA);
#elif SPI_CLK_PHASE == SPI_SETUP_LEADING_SAMPLE_TRAILING
    SET_BIT(Local_u8SPCR, SPCR_CPHA);
#else
#error "Wrong SPI_CLK_PHASE config"
#endif

    /*Clock Frequency */

#if SPI_CLK_FREQ == SPI_FREQ_4
    Local_u8SPCR &= SPI_CLK_SELECT_MASK;
    Local_u8SPCR |= SPI_FREQ_4;
#elif SPI_CLK_FREQ == SPI_FREQ_16
    Local_u8SPCR &= SPI_CLK_SELECT_MASK;
    Local_u8SPCR |= SPI_FREQ_16;
#elif SPI_CLK_FREQ == SPI_FREQ_64
    Local_u8SPCR &= SPI_CLK_SELECT_MASK;
    Local_u8SPCR |= SPI_FREQ_64;
#elif SPI_CLK_FREQ == SPI_FREQ_128
    Local_u8SPCR &= SPI_CLK_SELECT_MASK;
    Local_u8SPCR |= SPI_FREQ_128;
#elif SPI_CLK_FREQ == SPI_FREQ_DBL_2
    Local_u8SPCR &= SPI_CLK_SELECT_MASK;
    Local_u8SPCR |= SPI_FREQ_DBL_2;
    SET_BIT(SPSR, SPSR_SPI2X);
#elif SPI_CLK_FREQ == SPI_FREQ_DBL_8
    Local_u8SPCR &= SPI_CLK_SELECT_MASK;
    Local_u8SPCR |= SPI_FREQ_DBL_8;
    SET_BIT(SPSR, SPSR_SPI2X);
#elif SPI_CLK_FREQ == SPI_FREQ_DBL_32
    Local_u8SPCR &= SPI_CLK_SELECT_MASK;
    Local_u8SPCR |= SPI_FREQ_DBL_32;
    SET_BIT(SPSR, SPSR_SPI2X);
#elif SPI_CLK_FREQ == SPI_FREQ_DBL_64
    Local_u8SPCR &= SPI_CLK_SELECT_MASK;
    Local_u8SPCR |= SPI_FREQ_DBL_64;
    SET_BIT(SPSR, SPSR_SPI2X);
#else
#error "Wrong SPI_CLK_FREQ config"
#endif

    /*SPI Enable/Disable */
#if SPI_STATUS == SPI_ENABLE
    SET_BIT(Local_u8SPCR, SPCR_SPE);
#elif SPI_STATUS == SPI_DISABLE
    CLR_BIT(Local_u8SPCR, SPCR_SPE);
#else
#error "Wrong SPI_STATUS config "
#endif

    SPCR = Local_u8SPCR;
}

void SPI_MasterInit(void)
{

    /*Set as Master and initialize */
    SPI_Init();
    SET_BIT(SPCR, SPCR_MSTR);
    
    /*Initialize pins */
    DIO_SetPinDirection(SPI_PORT, SPI_SS_PIN, DIO_PIN_OUTPUT);
    DIO_SetPinValue(SPI_PORT, SPI_SS_PIN, DIO_PIN_LOW);

    DIO_SetPinDirection(SPI_PORT, SPI_MOSI_PIN, DIO_PIN_OUTPUT);
    DIO_SetPinDirection(SPI_PORT, SPI_MISO_PIN, DIO_PIN_INPUT);
    DIO_SetPinDirection(SPI_PORT, SPI_SCK, DIO_PIN_OUTPUT);
}
void SPI_SlaveInit(void)
{
    /*Set as Slave and initialize */
    SPI_Init();
    CLR_BIT(SPCR, SPCR_MSTR);

    /*Initialize pins */
    DIO_SetPinDirection(SPI_PORT, SPI_SS_PIN, DIO_PIN_INPUT);
    DIO_SetPinDirection(SPI_PORT, SPI_MOSI_PIN, DIO_PIN_INPUT);
    DIO_SetPinDirection(SPI_PORT, SPI_MISO_PIN, DIO_PIN_OUTPUT);
    DIO_SetPinDirection(SPI_PORT, SPI_SCK, DIO_PIN_INPUT);
}
u8 SPI_Transceive(u8 transmitData)
{

    /* Send Data */
    SPDR = transmitData;
    /* Wait till transfer is complete*/
    while (GET_BIT(SPSR, SPSR_SPIF) == 0);
    _delay_ms(2);
    /* Receive Data*/
    return SPDR;
}


 SPI_Status_T SPI_Tranceive_NoBlock(u8*data )
 {
	 u8 SPI_data=SPDR;
	 SPI_Status_T status ;
	 if(SPI_data==0){
		 SPDR=*data;
		 status=Send_NoBlock;
	 }
	 else{
		 *data=SPI_data;
		 SPDR=0;
		 status =Recive_NoBlock;
	 }
	 return status ;
 }
 
void SPI_SetCallBack(void (*fptr)(void))
{
    SPI_g_Callback = fptr;
}

ISR(SPI_STC_vect)
{
    if (SPI_g_Callback != NULL_PTR)
    {
        SPI_g_Callback();
    }
}
