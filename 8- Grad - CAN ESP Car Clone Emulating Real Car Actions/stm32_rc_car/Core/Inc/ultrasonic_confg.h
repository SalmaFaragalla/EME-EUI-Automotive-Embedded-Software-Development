#ifndef ULTRASONIC_CONFG_H_
#define ULTRASONIC_CONFG_H_

/* the 2 ultrasonic use Timer 1 through channel 3 and 2
*forward use channel 3 and back uses channel 2
*/

#define    TRIGER_FORWARD_PORT   GPIOB
#define    TRIGER_BACKWARD_PORT  GPIOB
#define    TRIGER_FORWARD_PIN   GPIO_PIN_0
#define    TRIGER_BACKWARD_PIN  GPIO_PIN_1
#define    CRACH_DISTANCE       15
#endif //ULTRASONIC_CONFG_H_
