/*
 *Author: Salma Faragalla
 */
#include "BIT_MATH.h"

#include "DIO_interface.h"
#include "ATMega32A_Registers.h"

enu_dio_error_status_t DIO_SetPinDirection(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx, enu_dio_pin_dir_t DIO_PIN_Dir)
{
    enu_dio_error_status_t ret_dio_error_status = DIO_OK;

    if ((DIO_PORTx > DIO_PORT_TOTAL) || (DIO_PINx > DIO_PIN_TOTAL) || (DIO_PIN_Dir > DIO_PIN_DIR_TOTAL))
    {
        ret_dio_error_status = DIO_NOK;
    }
    else
    { /*Do Nothing */
    }

    if (ret_dio_error_status == DIO_OK)
    {
        switch (DIO_PIN_Dir)
        {
        case DIO_PIN_OUTPUT:

            switch (DIO_PORTx)
            {
            case DIO_PORTA:
                SET_BIT(DDRA, DIO_PINx);
                break;

            case DIO_PORTB:
                SET_BIT(DDRB, DIO_PINx);
                break;

            case DIO_PORTC:
                SET_BIT(DDRC, DIO_PINx);
                break;

            case DIO_PORTD:
                SET_BIT(DDRD, DIO_PINx);
                break;

            default:
                ret_dio_error_status = DIO_NOK;
                break;
            }
            break;

        case DIO_PIN_INPUT:

            switch (DIO_PORTx)
            {
            case DIO_PORTA:
                CLR_BIT(DDRA, DIO_PINx);
                break;

            case DIO_PORTB:
                CLR_BIT(DDRB, DIO_PINx);
                break;

            case DIO_PORTC:
                CLR_BIT(DDRC, DIO_PINx);
                break;

            case DIO_PORTD:
                CLR_BIT(DDRD, DIO_PINx);
                break;

            default:
                ret_dio_error_status = DIO_NOK;
                break;
            }
        default:
            ret_dio_error_status = DIO_NOK;
            break;
        }
    }
    else
    { /*Do Nothing */
    }
    return ret_dio_error_status;
}
enu_dio_error_status_t DIO_SetPinValue(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx, enu_dio_pin_val_t DIO_PIN_Val)
{
    enu_dio_error_status_t ret_dio_error_status = DIO_OK;
    if ((DIO_PORTx > DIO_PORT_TOTAL) || (DIO_PINx > DIO_PIN_TOTAL) || (DIO_PIN_Val > DIO_PIN_VAL_TOTAL))
    {
        ret_dio_error_status = DIO_NOK;
    }
    else
    { /*Do Nothing */
    }
    if (ret_dio_error_status == DIO_OK)
    {
        switch (DIO_PIN_Val)
        {
        case DIO_PIN_HIGH:

            switch (DIO_PORTx)
            {
            case DIO_PORTA:
                SET_BIT(PORTA, DIO_PINx);
                break;

            case DIO_PORTB:
                SET_BIT(PORTB, DIO_PINx);
                break;

            case DIO_PORTC:
                SET_BIT(PORTC, DIO_PINx);
                break;

            case DIO_PORTD:
                SET_BIT(PORTD, DIO_PINx);
                break;

            default:
                ret_dio_error_status = DIO_NOK;
                break;
            }
            break;

        case DIO_PIN_LOW:

            switch (DIO_PORTx)
            {
            case DIO_PORTA:
                CLR_BIT(PORTA, DIO_PINx);
                break;

            case DIO_PORTB:
                CLR_BIT(PORTB, DIO_PINx);
                break;

            case DIO_PORTC:
                CLR_BIT(PORTC, DIO_PINx);
                break;

            case DIO_PORTD:
                CLR_BIT(PORTD, DIO_PINx);
                break;

            default:
                ret_dio_error_status = DIO_NOK;
                break;
            }
        default:
            ret_dio_error_status = DIO_NOK;
            break;
        }
    }
    else
    { /*Do Nothing */
    }
    return ret_dio_error_status;
}
enu_dio_error_status_t DIO_GetPinValue(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx, enu_dio_pin_val_t *ptr_DIO_PIN_Val)
{

    enu_dio_error_status_t ret_dio_error_status = DIO_OK;
    if ((DIO_PORTx > DIO_PORT_TOTAL) || (DIO_PINx > DIO_PIN_TOTAL))
    {
        ret_dio_error_status = DIO_NOK;
    }
    else
    { /*Do Nothing */
    }
    if (ret_dio_error_status == DIO_OK)
    {
        switch (DIO_PORTx)
        {
        case DIO_PORTA:
            *ptr_DIO_PIN_Val = GET_BIT(PINA, DIO_PINx);
            break;
        case DIO_PORTB:
            *ptr_DIO_PIN_Val = GET_BIT(PINB, DIO_PINx);
            break;
        case DIO_PORTC:
            *ptr_DIO_PIN_Val = GET_BIT(PINC, DIO_PINx);
            break;
        case DIO_PORTD:
            *ptr_DIO_PIN_Val = GET_BIT(PIND, DIO_PINx);
            break;
        default:
            ret_dio_error_status = DIO_NOK;
            break;
        }
    }
    else
    { /*Do Nothing */
    }
    return ret_dio_error_status;
}
enu_dio_error_status_t DIO_TogglePinValue(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx)
{
    enu_dio_error_status_t ret_dio_error_status = DIO_OK;

    if ((DIO_PORTx > DIO_PORT_TOTAL) || (DIO_PINx > DIO_PIN_TOTAL))
    {
        ret_dio_error_status = DIO_NOK;
    }
    else
    { /*Do Nothing */
    }
    if (ret_dio_error_status == DIO_OK)
    {
        switch (DIO_PORTx)
        {
        case DIO_PORTA:
            TOG_BIT(PORTA, DIO_PINx);
            break;
        case DIO_PORTB:
            TOG_BIT(PORTB, DIO_PINx);
            break;
        case DIO_PORTC:
            TOG_BIT(PORTC, DIO_PINx);
            break;
        case DIO_PORTD:
            TOG_BIT(PORTD, DIO_PINx);
            break;
        default:
            ret_dio_error_status = DIO_NOK;
            break;
        }
    }
    else
    { /*Do Nothing */
    }
    return ret_dio_error_status;
}
enu_dio_error_status_t DIO_SetPullUp(enu_dio_port_t DIO_PORTx, enu_dio_pin_t DIO_PINx)
{

    enu_dio_error_status_t ret_dio_error_status = DIO_OK;
    if ((DIO_PORTx > DIO_PORT_TOTAL) || (DIO_PINx > DIO_PIN_TOTAL))
    {
        ret_dio_error_status = DIO_NOK;
    }
    else
    { /*Do Nothing */
    }
    if (ret_dio_error_status == DIO_OK)
    {
        SET_BIT(DIO_PORTx, DIO_PINx);
    }
    else
    { /*Do Nothing */
    }
    return ret_dio_error_status;
}
enu_dio_error_status_t DIO_SetPortDirection(enu_dio_port_t DIO_PORTx, u8 DIO_Port_Dir)
{
    enu_dio_error_status_t ret_dio_error_status = DIO_OK;
    if ((DIO_PORTx > DIO_PORT_TOTAL) || (DIO_Port_Dir > DIO_MAX_REG_VALUE))
    {
        ret_dio_error_status = DIO_NOK;
    }
    else
    { /*Do Nothing */
    }
    if (ret_dio_error_status == DIO_OK)
    {
        switch (DIO_PORTx)
        {
        case DIO_PORTA:
            DDRA = DIO_Port_Dir;
            break;
        case DIO_PORTB:
            DDRB = DIO_Port_Dir;
            break;
        case DIO_PORTC:
            DDRC = DIO_Port_Dir;
            break;
        case DIO_PORTD:
            DDRD = DIO_Port_Dir;
            break;
        default:
            ret_dio_error_status = DIO_NOK;
            break;
        }
    }
    else
    { /*Do Nothing */
    }
    return ret_dio_error_status;
}
enu_dio_error_status_t DIO_SetPortValue(enu_dio_port_t DIO_PORTx, u8 DIO_PORT_Val)
{
    enu_dio_error_status_t ret_dio_error_status = DIO_OK;

    if ((DIO_PORTx > DIO_PORT_TOTAL) || (DIO_PORT_Val > DIO_MAX_REG_VALUE))
    {
        ret_dio_error_status = DIO_NOK;
    }
    else
    { /*Do Nothing */
    }
    if (ret_dio_error_status == DIO_OK)
    {
        switch (DIO_PORTx)
        {
        case DIO_PORTA:
            PORTA = DIO_PORT_Val;
            break;
        case DIO_PORTB:
            PORTB = DIO_PORT_Val;
            break;
        case DIO_PORTC:
            PORTC = DIO_PORT_Val;
            break;
        case DIO_PORTD:
            PORTD = DIO_PORT_Val;
            break;
        default:
            ret_dio_error_status = DIO_NOK;
            break;
        }
    }
    else
    { /*Do Nothing */
    }
    return ret_dio_error_status;
}
enu_dio_error_status_t DIO_GetPortValue(enu_dio_port_t DIO_PORTx, u8 *ptr_DIO_PORT_Val)
{
    enu_dio_error_status_t ret_dio_error_status = DIO_OK;
    switch (DIO_PORTx)
    {
    case DIO_PORTA:
        *ptr_DIO_PORT_Val = PINA;
        break;
    case DIO_PORTB:
        *ptr_DIO_PORT_Val = PINB;
        break;
    case DIO_PORTC:
        *ptr_DIO_PORT_Val = PINC;
        break;
    case DIO_PORTD:
        *ptr_DIO_PORT_Val = PIND;
        break;
    default:
        ret_dio_error_status = DIO_NOK;
        break;
    }
    return ret_dio_error_status;
}
