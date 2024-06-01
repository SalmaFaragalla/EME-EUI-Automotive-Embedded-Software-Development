/*
 *  Author: Salma Faragalla
 */
#ifndef DIO_INTERFACE_H_
#define DIO_INTERFACE_H_

/*******************************************************************************
 *                                Includes                                     *
 *******************************************************************************/
#include "STD_TYPES.h"

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
#define DIO_MAX_REG_VALUE 255

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
typedef enum
{
    DIO_PORTA = 0,
    DIO_PORTB,
    DIO_PORTC,
    DIO_PORTD,
    DIO_PORT_TOTAL
} enu_dio_port_t;

typedef enum
{
    DIO_PIN0 = 0,
    DIO_PIN1,
    DIO_PIN2,
    DIO_PIN3,
    DIO_PIN4,
    DIO_PIN5,
    DIO_PIN6,
    DIO_PIN7,
    DIO_PIN_TOTAL
} enu_dio_pin_t;

typedef enum
{
    DIO_OK = 0,
    DIO_NOK
} enu_dio_error_status_t;

typedef enum
{
    DIO_PIN_INPUT = 0,
    DIO_PIN_OUTPUT,
    DIO_PIN_DIR_TOTAL
} enu_dio_pin_dir_t;

typedef enum
{
    DIO_PIN_LOW = 0,
    DIO_PIN_HIGH,
    DIO_PIN_VAL_TOTAL
} enu_dio_pin_val_t;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * @Fn			                        :   DIO_SetPinDirection
 * @brief                               :   Sets a pin's direction to output or input
 *
 * @param[in]   DIO_PORTx               :   Pin's port number        (DIO_PORTA to DIO_PORTD)
 *              DIO_PINx                :   Pin's number             (DIO_PIN0 to DIO_PIN7)
 *              DIO_PIN_DIR             :   Required pin's direction (DIO_PIN_OUTPUT OR DIO_PIN_INPUT)
 *
 * @return      DIO_OK                  :   In case of Successful Operation
 *              DIO_NOK                 :   In case of Failed Operation
 */
enu_dio_error_status_t DIO_SetPinDirection(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx, enu_dio_pin_dir_t DIO_PIN_Dir);

/*
 * @Fn			                        :   DIO_SetPinValue
 * @brief                               :   Sets a pin's value to high or low
 *
 * @param[in]   DIO_PORTx               :   Pin's port number        (DIO_PORTA to DIO_PORTD)
 *              DIO_PINx                :   Pin's number             (DIO_PIN0 to DIO_PIN7)
 *              DIO_PIN_Val             :   Required pin's value     (DIO_PIN_HIGH OR DIO_PIN_LOW)
 *
 * @return      DIO_OK                  :   In case of Successful Operation
 *              DIO_NOK                 :   In case of Failed Operation
 */
enu_dio_error_status_t DIO_SetPinValue(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx, enu_dio_pin_val_t DIO_PIN_Val);

/*
 * @Fn			                        :   DIO_GetPinValue
 * @brief                               :   Gets a pin's value
 *
 * @param[in]   DIO_PORTx               :   Pin's port number        (DIO_PORTA to DIO_PORTD)
 *              DIO_PINx                :   Pin's number             (DIO_PIN0 to DIO_PIN7)
 *              ptr_DIO_PIN_Val         :   Pointer to the variable that will return the pin value
 *
 * @return      DIO_OK                  :   In case of Successful Operation
 *              DIO_NOK                 :   In case of Failed Operation
 */
enu_dio_error_status_t DIO_GetPinValue(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx, enu_dio_pin_val_t *ptr_DIO_PIN_Val);

/*
 * @Fn			                        :   DIO_TogglePinValue
 * @brief                               :   Toggles a pin's value
 *
 * @param[in]   DIO_PORTx               :   Pin's port number        (DIO_PORTA to DIO_PORTD)
 *              DIO_PINx                :   Pin's number             (DIO_PIN0 to DIO_PIN7)
 *
 * @return      DIO_OK                  :   In case of Successful Operation
 *              DIO_NOK                 :   In case of Failed Operation
 */
enu_dio_error_status_t DIO_TogglePinValue(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx);

/*
 * @Fn			                        :   DIO_SetPullUp
 * @brief                               :   Enables a pin's pull up resistor 
 *
 * @param[in]   DIO_PORTx               :   Pin's port number        (DIO_PORTA to DIO_PORTD)
 *              DIO_PINx                :   Pin's number             (DIO_PIN0 to DIO_PIN7)
 *
 * @return      DIO_OK                  :   In case of Successful Operation
 *              DIO_NOK                 :   In case of Failed Operation
 */
enu_dio_error_status_t DIO_SetPullUp(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx);

/*
 * @Fn			                        :   DIO_SetPortDirection
 * @brief                               :   Sets a port's direction
 *
 * @param[in]   DIO_PORTx               :   Port's number              (DIO_PORTA to DIO_PORTD)
 *              DIO_Port_Dir            :   Port's Direction
 *
 * @return      DIO_OK                  :   In case of Successful Operation
 *              DIO_NOK                 :   In case of Failed Operation
 */
enu_dio_error_status_t DIO_SetPortDirection(enu_dio_port_t DIO_PORTx, u8 DIO_Port_Dir);

/*
 * @Fn			                        :   DIO_SetPortValue
 * @brief                               :   Sets a port's value
 *
 * @param[in]   DIO_PORTx               :   Port's number              (DIO_PORTA to DIO_PORTD)
 *              DIO_PORT_Val            :   Port's value
 *
 * @return      DIO_OK                  :   In case of Successful Operation
 *              DIO_NOK                 :   In case of Failed Operation
 */
enu_dio_error_status_t DIO_SetPortValue(enu_dio_port_t DIO_PORTx, u8 DIO_PORT_Val);

/*
 * @Fn			                        :   DIO_GetPortValue
 * @brief                               :   Gets a port's value
 *
 * @param[in]   DIO_PORTx               :   Port's number              (DIO_PORTA to DIO_PORTD)
 *              DIO_PINx                :   Port's value
 *              ptr_DIO_PORT_Val        :   Pointer to the variable that will return the port value
 *
 * @return      DIO_OK                  :   In case of Successful Operation
 *              DIO_NOK                 :   In case of Failed Operation
 */
enu_dio_error_status_t DIO_GetPortValue(enu_dio_port_t DIO_PORTx, u8 *ptr_DIO_PORT_Val);

#endif /* DIO_INTERFACE_H_ */
