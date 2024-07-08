#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "imu.h"
#include "pid.h"
pid_paramer_t imu_temp_pid_data_para =
{
    .integrate_max = 30,
    .kp = 30,
    .ki = 0.5,
    .kd = 0,
    .control_output_limit = 100
};
pid_data_t imu_temp_pid_data;
/**********************************************************************************************************
*�� �� ��: imu_temp_pid_data_init
*����˵��: ���������¿���PID��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void imu_temp_pid_data_init(void)
{
    imu_temp_pid_data.expect = 0;
    imu_temp_pid_data.feedback = 0;

    imu_temp_pid_data.err = 0;
    imu_temp_pid_data.last_err = 0;
    imu_temp_pid_data.integrate = 0;
    imu_temp_pid_data.dis_err = 0;

    imu_temp_pid_data.control_output = 0;

    imu_temp_pid_data.short_circuit_flag = 0;
    imu_temp_pid_data.err_callback = NULL;
    imu_temp_pid_data.pri_data = NULL;
}

/**********************************************************************************************************
*�� �� ��: imu_temp_gpio_init
*����˵��: ���������¿���IO��ʼ�����û�����д�˺���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void imu_temp_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //ʹ��GPIOʱ��
    __HAL_RCC_GPIOC_CLK_ENABLE();

    //��ʼ��IO����
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void temp_imu_task(void const * argument)
{
    uint8_t count;

//    imu_temp_gpio_init();
    imu_temp_pid_data_init();
    while(1)
    {
//        //�¶�PID����
//        if (count == 100)
//        {
//            imu_temp_pid_data.feedback = tempDataFilter;
//            imu_temp_pid_data.expect = 50;
//            pid_control(&imu_temp_pid_data, &imu_temp_pid_data_para);
//        }

//        //ģ��PWM���
//        if (count == 100)
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
//        else if (count >= imu_temp_pid_data.control_output)
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

//        if (count == 100)
//            count = 0;
//        else
//            count++;

//        //˯��1ms
        vTaskDelay(1);
    }
}
