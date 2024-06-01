/*
 * EEPROM_Interface.h
 *
 * Created: 9/3/2023 10:22:04 AM
 *  Author: nada
 */ 


#ifndef EEPROM_INTERFACE_H_
#define EEPROM_INTERFACE_H_


#include "STD_TYPES.h"
typedef enum {
	NOERROR,
	EEPROM_Read_Error,/* 0*/
	EEPROM_write_Error,/*1*/
	EEPROM_Error_start=17,
	EEPROM_Error_writeadrees,/*18*/
	EEPROM_Error_byte,/*19*/
	EEPROM_Error_data,/*20*/
	EEPROM_Error_Repeatedstart,
	EEPROM_Read_Address_Error
}EEPROM_EROOR_T;

u8 EEPROM_u8ReadDataByte(u16 Copy_u16LocationAddress);

#endif /* EEPROM_INTERFACE_H_ */
