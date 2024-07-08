#include "avoid_obs.h"
#include "time_cnt.h"

#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "chassis.h"
#include "chassis_control.h"

#define THRESHOLD 2000

#define US
uint16_t raw_data, distance;

static Testime time_obj;
static uint8_t wait_fall = 0;
static GPIO_InitTypeDef US_GPIO_InitStruct = {0};

uint16_t distance_convert(uint16_t raw_data)
{
#ifdef US
    //����ʹ�õ��ͺŽ����޸�
    return raw_data * (0.340) / 2; //ת���ɺ��׵�λ
#else
    //����ʵ�ֶ������ͺŵļ���
#ifdef SR                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
    return 0;
#endif
#endif
}

/**********************************************************************
 * @Name    avoid_init
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void avoid_init(void)
{
    //���ú����ŵ���Ϣ
    HAL_GPIO_WritePin(US_RECEIVE_GPIO_Port, US_RECEIVE_Pin, GPIO_PIN_RESET);
    US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
    US_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    US_GPIO_InitStruct.Pull = GPIO_PULLUP;
    US_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
}

/**********************************************************************
 * @Name    avoid_callback
 * @declaration : �����жϺ���
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void avoid_callback(void)
{
    if (!wait_fall) //���ȵȵ�����������
    {
        wait_fall = 1;              //��һ�������½���
        Get_Time_Period(&time_obj); //��ȡ��ǰʱ��
        /* �������Ŷ��½��ش���*/
        US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
        US_GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        US_GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
    }
    else
    {
        //�����½���
        Get_Time_Period(&time_obj);            //����ʱ��
        raw_data = time_obj.Time_Delta;        //��ȡʱ���
        distance = distance_convert(raw_data); //��ԭʼ����ת���õ���������
        if (distance > THRESHOLD)              //̫�󣬳���
            distance = 0;
        /*���������û������ز���*/
        US_GPIO_InitStruct.Pin = US_RECEIVE_Pin;
        US_GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        US_GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(US_RECEIVE_GPIO_Port, &US_GPIO_InitStruct);
        wait_fall = 0;
    }
}

/**********************************************************************
 * @Name    start_avoid
 * @declaration : ���е��εĲ��
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void start_avoid(void)
{
    HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(US_SEND_GPIO_Port, US_SEND_Pin, GPIO_PIN_RESET);
    while (distance == 0)
        osDelay(10); //����ʽ
    return;
}

#define DISTANCE_THRESHOLD 0XDB
#define AVOID_OBS_LOW_SPEED 120
#include "chassis_control.h"

//����1���� 2����
void Wait_For_Avoid(int dir)
{
    Comfirm_Online(dir);
    short dir_flag = 0;
    //��ɫ�볡�ı��� ������������
    if (dir == 1)
    {
        dir_flag = 1;
    }
    else if (dir == 2)
    {
        dir_flag = -1;
    }
    set_speed(0, AVOID_OBS_LOW_SPEED, 0);
    while (distance > 0XDB)
        osDelay(5);
    set_speed(0, 0, 0);
    osDelay(100);
    move_by_encoder(1, 15 * dir_flag);
    Wait_OKInf(2, 5000);
    move_by_encoder(2, 60);
    Wait_OKInf(2, 5000);
    move_by_encoder(1, -15 * dir_flag);
    Wait_OKInf(2, 5000);
    Comfirm_Online(dir);
}
int Get_Distance(void)
{
    return distance;
}

