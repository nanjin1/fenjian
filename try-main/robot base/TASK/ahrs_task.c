#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "ahrs.h"
#include "imu.h"
void ahrs_imu_task_(void const * argument)
{
     portTickType xLastWakeTime;
	
	imu_init();
	ahrs_init();
    
    //��ȡϵͳ��ǰ����
    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        //��ȡimu����
		get_imu_data();
        //��̬����
		ahrs_update();
        //˯��5ms
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
    vTaskDelete(NULL);//��ֹ����
}