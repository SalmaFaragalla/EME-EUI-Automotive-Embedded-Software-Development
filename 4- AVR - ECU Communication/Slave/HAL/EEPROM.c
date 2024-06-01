/*
 * EEPROM.c
 *
 * Created: 9/3/2023 10:19:34 AM
 *  Author: nada
 */ 
#define F_CPU 16000000UL
#include <util/delay.h>
#include "EEPROM_CFG.h"
#include "EEPROM_Interface.h"
#include "EEPROM_Private.h"
#include "STD_TYPES.h"
#include "TWI_interface.h"


 EEPROM_EROOR_T EEPROM_voidSendDataByte(u16 Copy_u16LocationAddress, u8 Copy_u8DataByte)
    {
	    u8 Local_u8AddressPacket;
	    EEPROM_EROOR_T status=NOERROR ;
	  /*To select the sector*/
	  	Local_u8AddressPacket =Fixed_Address | (EEPROM_Adress_CFG<<2) | (u8)(Copy_u16LocationAddress >>8);

	    /*Send start condition*/
	    if(TWI_SendStartCondition()!=NoError)
	    {
		    status=EEPROM_Error_start;
	    }
	    else
	    {

		    /*Send the address packet*/
		    if(TWI_SendSlaveAddressWithWrite(Local_u8AddressPacket)!=NoError)
		    {
			    status=EEPROM_Error_writeadrees;
		    }
		    else
		    {

			    /*Send the rest 8bits of the location address*/
			    if(TWI_MasterWriteDataByte((u8)Copy_u16LocationAddress)!=NoError)
			    {
				    status=EEPROM_Error_byte;
			    }
			    else
			    {

				    /*Send the data byte to the memory location*/
				    if(TWI_MasterWriteDataByte(Copy_u8DataByte)!=NoError)
				    {
					    status=EEPROM_Error_data;
				    }
			    }
		    }
	    }
	    /*Send stop condition*/
	    TWI_SendStopCondition();

	    /*Delay until the write cycle is finished*/
	    _delay_ms(10);
	    return status;
    }

u8 EEPROM_u8ReadDataByte(u16 Copy_u16LocationAddress)
    {
	    u8 Local_u8AddressPacket,Local_u8Data;
	    EEPROM_EROOR_T status =NOERROR;
	    /*To select the sector*/
	  Local_u8AddressPacket =Fixed_Address | (EEPROM_Adress_CFG<<2) | (u8)(Copy_u16LocationAddress >>8);


	    /*Send start condition*/
	    if(TWI_SendStartCondition()!=NoError)
	    {
		    status=EEPROM_Error_start;
	    }
	    else
	    {
		    /*Send the address packet with write request*/
		    if(TWI_SendSlaveAddressWithWrite(Local_u8AddressPacket)!=NoError)
		    {
			    status==EEPROM_Error_writeadrees;
		    }
		    else
		    {




			    /*Send the rest 8bits of the location address*/
			    if(TWI_MasterWriteDataByte((u8)Copy_u16LocationAddress)!=NoError)
			    {
				    status=EEPROM_Error_byte;
			    }
			    else
			    {



				    /*Send repeated start to change write request into read request*/
				    if(TWI_SendRepeatedStart()!=NoError)
				    {
					    status=EEPROM_Error_Repeatedstart;
				    }
				    else
				    {
					    /*Send the address packet with read request*/
					    if(TWI_SendSlaveAddressWithRead(Local_u8AddressPacket)!=NoError)
					    {
						    status=EEPROM_Read_Address_Error;
					    }
					    else
					    {
						    /*Get the data from memory*/
						    if(TWI_MasterReadDataByte(&Local_u8Data)!=NoError)
						    {
							    status=EEPROM_Read_Error;
						    }
					    }
				    }
			    }
		    }
	    }

	    /*send the stop condition*/
	    TWI_SendStopCondition();

	    return Local_u8Data;
    }
