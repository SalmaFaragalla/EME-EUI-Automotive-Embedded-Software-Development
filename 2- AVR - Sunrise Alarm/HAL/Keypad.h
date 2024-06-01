/*
 * Keypad.h
 *
 * Created: 19-Aug-23 7:53:48 PM

 */ 


#ifndef KEYPAD_H_
#define KEYPAD_H_
/*******************************************************************************
 *                                Includes                                     *
 *******************************************************************************/
#include "Keypad_Cfg.h"

typedef enum{
	PRESSED,
	NOT_PRESSED
	}KEYPAD_ERROR_T;

/*******************************************************************************
 *                              Functions Prototypes                           *
 *******************************************************************************/

/*
 * @Fn			                        :   keyPad_Init
 * @brief                               :   Initializes a keypad
 *
 * @param[in]   void                      
 *
 * @return      void                   
 *                                
 */	
void keyPad_Init (void);

/*
 * @Fn			                        :   KeyPad_GetValue
 * @brief                               :   Gets the value of the pressed key
 *
 * @param[in]   void                      
 *
 * @return      PRESSED                     :   In case of Successful Operation 
 *              NOT_PRESSED                 :   In case of Failed Operation
 *                                
 */
KEYPAD_ERROR_T  KeyPad_GetValue(void);

#endif /* KEYPAD_H_ */