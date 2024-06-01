/*
 * Author: Salma Faragalla
 * File: SPI_private.h
 * Layer: MCAL
 * Created: 01-Sep-23 12:10:33 PM
 * 
 */ 
#ifndef SPI_PRIVATE_H_
#define SPI_PRIVATE_H_

#include "STD_TYPES.h"

#define SPCR * ((volatile u8 *)(0X2D))

#define SPCR_SPIE 7 
#define SPI_INT_DISABLE					0
#define SPI_INT_ENABLE 					1

#define SPCR_SPE 6
#define SPI_DISABLE								0
#define SPI_ENABLE 								1

#define SPCR_DORD 5
#define SPI_DATA_LSB_FIRST 						0
#define SPI_DATA_MSB_FIRST 						1

#define SPCR_MSTR 4

#define SPCR_CPOL 3
#define SPI_RISING_LEADING_FALLING_TRAILING 	0
#define SPI_FALLING_LEADING_RISING_TRAILING		1




#define SPCR_CPHA 2
#define SPI_SAMPLE_LEADING_SETUP_TRAILING 		0
#define SPI_SETUP_LEADING_SAMPLE_TRAILING		1

#define SPI_CLK_SELECT_MASK 0b11111100
#define SPI_FREQ_4					0
#define SPI_FREQ_16					1
#define SPI_FREQ_64					2
#define SPI_FREQ_128					3
#define SPI_FREQ_DBL_2					0
#define SPI_FREQ_DBL_8					1
#define SPI_FREQ_DBL_32					2
#define SPI_FREQ_DBL_64			    	3

#define SPSR * ((volatile u8 *)(0X2E))
#define SPSR_SPIF 7
#define SPSR_WCOL 6
#define SPSR_SPI2X 0


#define SPDR * ((volatile u8 *)(0X2F))

#define SPI_IDLE 0
#define SPI_BUSY 1


#define NULLPTR  ((void*)0x0)
#define NULL_PTR	NULLPTR
#endif /* SPI_PRIVATE_H_ */
