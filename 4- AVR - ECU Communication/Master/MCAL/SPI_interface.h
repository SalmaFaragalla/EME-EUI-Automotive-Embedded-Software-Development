/*
 * SPI_interface.h
 *
 * Created: 01-Sep-23 12:54:02 PM
 *  Author: Salma
 */ 


#ifndef SPI_INTERFACE_H_
#define SPI_INTERFACE_H_

#include "STD_TYPES.h"

void SPI_MasterInit(void);
void SPI_SlaveInit(void);

u8 SPI_Transceive(u8 transmitData);

void SPI_SetCallBack(void(*fptr)(void));
typedef enum {
	Send_NoBlock,  // while using interrupt
	Recive_NoBlock
	
}SPI_Status_T;
//Interrupt
SPI_Status_T SPI_Tranceive_NoBlock(u8*data) ;
#endif /* SPI_INTERFACE_H_ */
