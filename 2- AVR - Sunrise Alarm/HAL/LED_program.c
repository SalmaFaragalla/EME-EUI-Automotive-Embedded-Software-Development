/*
 *Author: Salma Faragalla
 */

#include "DIO_interface.h"
#include "LED_interface.h"
#include "LED_config.h"

enu_led_error_status_t LED_Init(enu_led_color_t led_color)
{
    enu_led_error_status_t ret_led_error_status = LED_OK;
    if (led_color > LED_TOTAL)
    {
        ret_led_error_status = LED_NOK;
        return ret_led_error_status;
    }

    switch (led_color)
    {
    case LED_GREEN:
        DIO_SetPinDirection(LED_GREEN_PORT, LED_GREEN_PIN, DIO_PIN_OUTPUT);
        break;

    case LED_RED:
        DIO_SetPinDirection(LED_RED_PORT, LED_RED_PIN, DIO_PIN_OUTPUT);
        break;

    case LED_BLUE:
        DIO_SetPinDirection(LED_BLUE_PORT, LED_BLUE_PIN, DIO_PIN_OUTPUT);
        break;

    case LED_YELLOW:
        DIO_SetPinDirection(LED_YELLOW_PORT, LED_YELLOW_PIN, DIO_PIN_OUTPUT);
        break;

    default:
        ret_led_error_status = LED_NOK;
        break;
    }
    return ret_led_error_status;
}

enu_led_error_status_t LED_TurnOn(enu_led_color_t led_color)
{
    enu_led_error_status_t ret_led_error_status = LED_OK;
    switch (led_color)
    {
    case LED_GREEN:
        DIO_SetPinValue(LED_GREEN_PORT, LED_GREEN_PIN, DIO_PIN_HIGH);
        break;

    case LED_RED:
        DIO_SetPinValue(LED_RED_PORT, LED_RED_PIN, DIO_PIN_HIGH);
        break;

    case LED_BLUE:
        DIO_SetPinValue(LED_BLUE_PORT, LED_BLUE_PIN, DIO_PIN_HIGH);
        break;

    case LED_YELLOW:
        DIO_SetPinValue(LED_YELLOW_PORT, LED_YELLOW_PIN, DIO_PIN_HIGH);
        break;
    default:
        ret_led_error_status = LED_NOK;
        break;
    }
    return ret_led_error_status;
}

enu_led_error_status_t LED_TurnOff(enu_led_color_t led_color)
{
    enu_led_error_status_t ret_led_error_status = LED_OK;
    switch (led_color)
    {
    case LED_GREEN:
        DIO_SetPinValue(LED_GREEN_PORT, LED_GREEN_PIN, DIO_PIN_LOW);
        break;

    case LED_RED:
        DIO_SetPinValue(LED_RED_PORT, LED_RED_PIN, DIO_PIN_LOW);
        break;

    case LED_BLUE:
        DIO_SetPinValue(LED_BLUE_PORT, LED_BLUE_PIN, DIO_PIN_LOW);
        break;

    case LED_YELLOW:
        DIO_SetPinValue(LED_YELLOW_PORT, LED_YELLOW_PIN, DIO_PIN_LOW);
        break;
    default:
        ret_led_error_status = LED_NOK;
        break;
    }
    return ret_led_error_status;
}

enu_led_error_status_t LED_Toggle(enu_led_color_t led_color)
{
    enu_led_error_status_t ret_led_error_status = LED_OK;
    switch (led_color)
    {
    case LED_GREEN:
        DIO_TogglePinValue(LED_GREEN_PORT, LED_GREEN_PIN);
        break;

    case LED_RED:
        DIO_TogglePinValue(LED_RED_PORT, LED_RED_PIN);
        break;

    case LED_BLUE:
        DIO_TogglePinValue(LED_BLUE_PORT, LED_BLUE_PIN);
        break;

    case LED_YELLOW:
        DIO_TogglePinValue(LED_YELLOW_PORT, LED_YELLOW_PIN);
        break;
    default:
        ret_led_error_status = LED_NOK;
        break;
    }
    return ret_led_error_status;
}
