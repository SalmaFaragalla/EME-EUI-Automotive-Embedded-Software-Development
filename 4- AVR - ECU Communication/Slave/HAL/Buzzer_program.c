/*
 *Author: Salma Faragalla
 */

#include "DIO_interface.h"
 
#include "Buzzer_interface.h"
#include "Buzzer_config.h"


enu_BUZZER_error_status_t BUZZER_Init(enu_BUZZER_id_t buzzer_id)
{
        enu_BUZZER_error_status_t ret_buzzer_error_status = BUZZER_OK;
        if (buzzer_id > BUZZER_TOTAL)
        {
            ret_buzzer_error_status = BUZZER_NOK;
            return ret_buzzer_error_status;
        }

        switch (buzzer_id)
        {
            case BUZZER1:
            DIO_SetPinDirection(BUZZER1_PORT, BUZZER1_PIN, DIO_PIN_OUTPUT);
            break;
            
            default:
            ret_buzzer_error_status = BUZZER_NOK;
            break;
        }
        
         return ret_buzzer_error_status;       
}
enu_BUZZER_error_status_t BUZZER_TurnOn(enu_BUZZER_id_t buzzer_id)
{
    enu_BUZZER_error_status_t ret_buzzer_error_status = BUZZER_OK;
    if (buzzer_id > BUZZER_TOTAL)
    {
        ret_buzzer_error_status = BUZZER_NOK;
        return ret_buzzer_error_status;
    }

    switch (buzzer_id)
    {
        case BUZZER1:
        DIO_SetPinValue(BUZZER1_PORT, BUZZER1_PIN, DIO_PIN_HIGH);
        break;
                    default:
                    ret_buzzer_error_status = BUZZER_NOK;
                    break;
    }
    return ret_buzzer_error_status;         
}
enu_BUZZER_error_status_t BUZZER_TurnOff(enu_BUZZER_id_t buzzer_id)
{
        enu_BUZZER_error_status_t ret_buzzer_error_status = BUZZER_OK;
        if (buzzer_id > BUZZER_TOTAL)
        {
            ret_buzzer_error_status = BUZZER_NOK;
            return ret_buzzer_error_status;
        }

        switch (buzzer_id)
        {
            case BUZZER1:
            DIO_SetPinValue(BUZZER1_PORT, BUZZER1_PIN, DIO_PIN_LOW);
            break;
            default:
            ret_buzzer_error_status = BUZZER_NOK;
            break;
        }
        return ret_buzzer_error_status;
}
