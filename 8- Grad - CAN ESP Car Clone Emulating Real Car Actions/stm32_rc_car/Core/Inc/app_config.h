/*
 * app_config.h
 *
 *  Created on: Oct 24, 2023
 *      Author: TEAM
 */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_


/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */

/* - uart processing */
#define APP_UART_LAST_RX_BYTE       rx_byte
#define APP_UART_RX_QUEUE_LENGTH    20
#define APP_UART_RX_QUEUE_ITEM_SIZE 1
#define APP_UART_DMA_RX_BUFFER_LENGTH 2

/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */

/* Speed Levels */
# define SPEED_ZERO  	((uint8_t)0)
# define SPEED_MIN  	((uint8_t)70)
# define SPEED_MED 	    ((uint8_t)80)
# define SPEED_HIGH 	((uint8_t)90)
# define SPEED_MAX      ((uint8_t)100)

/* SALMA END */

#endif /* INC_APP_CONFIG_H_ */
