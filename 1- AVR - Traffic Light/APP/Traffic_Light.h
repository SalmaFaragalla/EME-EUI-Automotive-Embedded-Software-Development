/*
 * Traffic_Light.h
 *
 * Created: 20-Aug-23 12:33:17 PM
 *  Author: Salma
 */ 


#ifndef TRAFFIC_LIGHT_H_
#define TRAFFIC_LIGHT_H_

typedef enum{
    
    CAR=5,
    
    Pedestrian=8
    
    
}TRAFFIC_MODE;




void traffic_light_Init(void);
void Delay_5sec(void);
void traffic_light_Runnable(void);
#endif /* TRAFFIC LIGHT_H_ */