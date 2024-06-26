
#ifndef TWI_INTERFACE_H_
#define TWI_INTERFACE_H_

typedef enum
{
	NoError,
	StartConditionErr,
	RepeatedStartError,
	SlaveAddressWithWriteErr,
	SlaveAddressWithReadErr,
	MasterWriteByteErr,
	MasterReadByteErr,
	Slave_InitFalied

	
}TWI_ErrStatus;

typedef enum {
	Slave_AddressRead,
	Slave_AddressWrite,
	
	}SLAVE_Address_T;
	
	
/*Set_Bitrate*/	
void TWI_SETBit_Rate(void); 	
/*Set master address to 0 if master will not be addressed*/
void TWI_voidInitMaster(u8 Copy_u8Address);

/*TWI_ErrStatus TWI_voidInitSlave(u8 Copy_u8Address);*/
TWI_ErrStatus TWI_voidInitSlave(u8 Copy_u8Address);
TWI_ErrStatus TWI_SendStartCondition(void);

TWI_ErrStatus TWI_SendRepeatedStart(void);

TWI_ErrStatus TWI_SendSlaveAddressWithWrite(u8 Copy_u8SlaveAddress);
TWI_ErrStatus TWI_SendSlaveAddressWithRead(u8 Copy_u8SlaveAddress);

TWI_ErrStatus TWI_MasterWriteDataByte(u8 Copy_u8DataByte);
TWI_ErrStatus TWI_MasterReadDataByte(u8* Copy_pu8DataByte);

void TWI_SendStopCondition(void);



#endif
