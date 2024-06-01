/*
 *Author: Salma Faragalla
 */

#ifndef LED_INTERFACE_H_
#define LED_INTERFACE_H_

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
typedef enum
{
    LED_GREEN = 0,
    LED_BLUE,
    LED_YELLOW,
    LED_RED,
    LED_TOTAL
} enu_led_color_t;

typedef enum
{
    LED_OK = 0,
    LED_NOK

} enu_led_error_status_t;

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
enu_led_error_status_t LED_Init(enu_led_color_t led_color);

/*
 * @Fn			                        :   LED_TurnOn
 * @brief                               :   Turns on a LED
 *
 * @param[in]   led_color               :   Led Color  (LED_GREEN,LED_BLUE,LED_YELLOW,LED_RED)
 *
 * @return      LED_OK                  :   In case of Successful Operation
 *              LED_NOK                 :   In case of Failed Operation
 */
enu_led_error_status_t LED_TurnOn(enu_led_color_t led_color);

/*
 * @Fn			                        :   LED_TurnOff
 * @brief                               :   Turns off a LED
 *
 * @param[in]   led_color               :   Led Color  (LED_GREEN,LED_BLUE,LED_YELLOW,LED_RED)
 *
 * @return      LED_OK                  :   In case of Successful Operation
 *              LED_NOK                 :   In case of Failed Operation
 */
enu_led_error_status_t LED_TurnOff(enu_led_color_t led_color);

/*
 * @Fn			                        :   LED_Toggle
 * @brief                               :   Toggles a LED
 *
 * @param[in]   led_color               :   Led Color  (LED_GREEN,LED_BLUE,LED_YELLOW,LED_RED)
 *
 * @return      LED_OK                  :   In case of Successful Operation
 *              LED_NOK                 :   In case of Failed Operation
 */
enu_led_error_status_t LED_Toggle(enu_led_color_t led_color);

#endif /* LED_INTERFACE_H_ */
