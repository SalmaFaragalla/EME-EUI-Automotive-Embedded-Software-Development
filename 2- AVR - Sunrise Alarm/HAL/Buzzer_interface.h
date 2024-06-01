/*
 *Author: Salma Faragalla
 */

#ifndef BUZZER_INTERFACE_H_
#define BUZZER_INTERFACE_H_

/*******************************************************************************
 *                               Types Declaration                             *
 *******************************************************************************/
typedef enum
{
    BUZZER1,
    BUZZER_TOTAL
}enu_BUZZER_id_t;

typedef enum
{
    BUZZER_OK,
    BUZZER_NOK
}enu_BUZZER_error_status_t;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * @Fn			                        :   BUZZER_Init
 * @brief                               :   Initializes a buzzer
 *
 * @param[in]   buzzer_id               :   Buzzer's id     (BUZZER1)
 *
 * @return      BUZZER_OK               :   In case of Successful Operation
 *              BUZZER_NOK              :   In case of Failed Operation
 */
enu_BUZZER_error_status_t BUZZER_Init(enu_BUZZER_id_t buzzer_id);

/*
 * @Fn			                        :   BUZZER_TurnOn
 * @brief                               :   Turns on a buzzer
 *
 * @param[in]   buzzer_id               :   Buzzer's id     (BUZZER1)
 *
 * @return      BUZZER_OK               :   In case of Successful Operation
 *              BUZZER_NOK              :   In case of Failed Operation
 */
enu_BUZZER_error_status_t BUZZER_TurnOn(enu_BUZZER_id_t buzzer_id);

/*
 * @Fn			                        :   BUZZER_TurnOff
 * @brief                               :   Turns off a buzzer
 *
 * @param[in]   buzzer_id               :   Buzzer's id     (BUZZER1)
 *
 * @return      BUZZER_OK               :   In case of Successful Operation
 *              BUZZER_NOK              :   In case of Failed Operation
 */
enu_BUZZER_error_status_t BUZZER_TurnOff(enu_BUZZER_id_t buzzer_id);


#endif /* BUZZER_INTERFACE_H_ */