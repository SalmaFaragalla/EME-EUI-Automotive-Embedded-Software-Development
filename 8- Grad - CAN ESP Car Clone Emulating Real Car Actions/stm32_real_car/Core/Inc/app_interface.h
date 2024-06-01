/*
 * app_interface.h
 *
 *  Created on: Oct 21, 2023
 *      Author: Hossam Elwahsh
 */

#ifndef INC_APP_INTERFACE_H_
#define INC_APP_INTERFACE_H_

/* Typedefs */

/* Struct for current lighting states */
typedef struct
{
	BOOLEAN bool_brake_lights		:	SIZE_ONE_BIT;
	BOOLEAN bool_front_lights		:	SIZE_ONE_BIT;
	BOOLEAN bool_reverse_lights		:	SIZE_ONE_BIT;
	BOOLEAN bool_left_indicator		:	SIZE_ONE_BIT;
	BOOLEAN bool_right_indicator	:	SIZE_ONE_BIT;
	BOOLEAN bool_reserved_1	        :	SIZE_ONE_BIT;
	BOOLEAN bool_reserved_2			:	SIZE_ONE_BIT;
	BOOLEAN bool_reserved_3			:	SIZE_ONE_BIT;
}st_lighting_state_t;

/* Enums for car transmission states */
typedef enum
{
	TRANSMISSION_STATE_NONE		=	ZERO	,
	TRANSMISSION_STATE_PARKING				,
	TRANSMISSION_STATE_REVERSE				,
	TRANSMISSION_STATE_NEUTRAL				,
	TRANSMISSION_STATE_DRIVE				,
	TRANSMISSION_STATE_TOTAL				,
}en_transmission_state_t;

/* Enum for different car steering mapped states */
typedef enum
{
	STEERING_STATE_NONE			= APP_ESP_DATA_STEERING_NONE            ,
	STEERING_STATE_STRAIGHT		= APP_ESP_DATA_STEERING_STRAIGHT		,
	STEERING_STATE_LEFT			= APP_ESP_DATA_STEERING_LEFT			,
	STEERING_STATE_SHARP_LEFT	= APP_ESP_DATA_STEERING_SHARP_LEFT		,
	STEERING_STATE_RIGHT		= APP_ESP_DATA_STEERING_RIGHT			,
	STEERING_STATE_SHARP_RIGHT	= APP_ESP_DATA_STEERING_SHARP_RIGHT		,
	STEERING_STATE_TOTAL
}en_steering_state_t;

/* Struct stores last sent data states to prevent duplicated/unnecessary sends */
typedef struct
{
	/* lighting value */
	uint32_t u32_lighting_val;

	/* throttle value */
	uint8_t u8_throttle_val;

	/* lighting value */
	st_lighting_state_t st_lighting_state;

	/* transmission state */
	en_transmission_state_t en_transmission_state;

	/* steering state */
	en_steering_state_t en_steering_state;

}st_last_data_state_t;

/* Struct bit by bit converter for lighting bytes sent by the CAR High Speed Can Bus */
typedef struct
{
	/* byte 1 */
	uint32_t u32_unused_bit0:1;
	uint32_t u32_front_light_bit:1;
	uint32_t u32_unused_bit2:1;
	uint32_t u32_unused_bit3:1;
	uint32_t u32_unused_bit4:1;
	uint32_t u32_unused_bit5:1;
	uint32_t u32_unused_bit6:1;
	uint32_t u32_unused_bit7:1;

	/* byte 2 */
	uint32_t u32_unused_bit8:1;
	uint32_t u32_unused_bit9:1;
	uint32_t u32_unused_bit10:1;
	uint32_t u32_unused_bit11:1;
    uint32_t u32_unused_bit12:1;
	uint32_t u32_unused_bit13:1;
	uint32_t u32_unused_bit14:1;
	uint32_t u32_left_indicator_bit:1;


	/* byte 3 */
	uint32_t u32_unused_bit16:1;
	uint32_t u32_unused_bit17:1;
    uint32_t u32_brake_lights_bit:1;
	uint32_t u32_unused_bit19:1;
	uint32_t u32_unused_bit20:1;
	uint32_t u32_right_indicator_bit:1;
	uint32_t u32_unused_bit22:1;
	uint32_t u32_unused_bit23:1;

	/* byte 4 */
	uint32_t u32_unused_bit24:1;
	uint32_t u32_unused_bit25:1;
	uint32_t u32_unused_bit26:1;
	uint32_t u32_unused_bit27:1;
	uint32_t u32_unused_bit28:1;
	uint32_t u32_unused_bit29:1;
	uint32_t u32_unused_bit30:1;
	uint32_t u32_unused_bit31:1;
}st_lighting_bits_t;


/* Converter union for CAN received data */
typedef union
{
	uint8_t 	RxData[APP_RX_DATA_LENGTH]	;	// Receive buffer
	uint8_t		u8_rxNumber					;
	uint16_t 	u16_rxNumber				;
	uint32_t 	u32_rxNumber				;
	st_lighting_bits_t u8_lighting_bits		;
}un_data_converter_t;

/* struct for CAN queued item to be processed */
typedef struct
{
	uint16_t 			item_id				;
	un_data_converter_t un_data_converter	;
}st_can_queued_item_t;

typedef uint8_t app_uart_queue_item_t;

#endif /* INC_APP_INTERFACE_H_ */
