
#include "Ultrasonic_Task.h"
#include "cmsis_os.h"


extern transmission_en  gl_transmission_en;
ULTRASONIC_PASS Pass_Signal;
static ULTRASONIC_Status Status=Time_Out_;
extern TaskHandle_t Ultrasonic_Timeout_Handel;
extern uint8_t gl_u8_throttle ;
extern SemaphoreHandle_t semaphore_transmissionHandle;
extern	SemaphoreHandle_t semaphore_OLEDHandle;
void Ultrasonic_Task (void*pvParameter )
    {

	  vTaskSuspend(NULL);

        for(;;)
        {

            /* starts timeout timer - resets ultrasonic flag / stage to ZERO */
        	 vTaskResume( Ultrasonic_Timeout_Handel);

            if(gl_transmission_en==Drive)
            {
                set_Ultrasonic_num(ULTRASONIC1);
            }
            else
            {
            	set_Ultrasonic_num(ULTRASONIC2);
            }
            Ultrasonic_Updatedistance();
            if(Status == Success_)
            {
                if(Ultrasonc_getdistace()<CRACH_DISTANCE )
                {
                    Pass_Signal= Red_Flag;
                     gl_u8_throttle = 0;
                     xSemaphoreGive(semaphore_transmissionHandle);
                     xSemaphoreGive(semaphore_OLEDHandle);
                }
                else
                {
                    Pass_Signal= Green_Flag;
                }
            }
            else if( Status == Time_Out_)
            {
                Pass_Signal= Green_Flag;
            }

            osDelay(350);
        }
    }



 void Ultrasonic_Timeout_Task(void*pvParameter)
    {
        vTaskSuspend(NULL);
        static uint8_t enterfunc_count=0;
        ULTRASONIC_STAGE stage;
        for(;;)
        {
        	if(enterfunc_count == 0){
        	enterfunc_count++;
               }
        	else if(enterfunc_count == 1)
            {

                stage = Ultrasonc_getstage();
                if(stage == Half_way_operation)
                {
                    Status = Time_Out_; //time_out cause the ultrasonic took more than 20ms that means the car doesn't detect any obstacle within 4 meter range
                    Ultrasonic_Int_Timeout();

                }
                else if (stage == Complete_operation)
                {

                    Status = Success_;

                }
                enterfunc_count=0;
                vTaskSuspend(NULL);
            }
            osDelay(20);
        }

    }


