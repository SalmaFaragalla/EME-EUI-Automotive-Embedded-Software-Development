/*
 * Vehicle_comm.h
 *
 * Created: 03-Sep-23 12:09:52 PM
 *  Author: Salma
 */ 


#ifndef VEHICLE_COMM_H_
#define VEHICLE_COMM_H_


#define MAX_SPEED_LIMIT 128
#define MAX_SPEED_R_LIMIT 30
#define MAX_SPEED_P_N_LIMIT 0

void APP_Init(void);
void APP_DisplayMainMenu(void);
void APP_RuntimeValues (void);
void Check_limit(void);
void APP_runnable (void);
void Set_limit(void);
#endif /* VEHICLE_COMM_H_ */
