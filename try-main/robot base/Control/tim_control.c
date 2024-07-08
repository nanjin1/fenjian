#include "tim_control.h"
#include "tim.h"
#include "sin.h"
double encoder_val[5] = {0}; //Ĭ��Ϊ0
short status_flag[5];        //

double encoder_sum = 0, temp_sum = 0; //������������صı���
int count_ = 0;
int rising_val[5], falling_val[5], direct_[5], update_count[5]; //���ת����ر���

double cap_temp_val[5];
short cap_cnt[5];
int first_flag[5];
int num_cnt[5];
extern TIM_HandleTypeDef htim1; //Ҫ�õ��Ķ�ʱ�����
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

#define FORWARD 1
#define BACKWARD -1
#define SPEED_PARAM 100000.0
#define TIM_COUNT_VAL 10000
#define FILTER 10
#define THRESHOLD_ 20.0

/*******************
 *@name:HAL_TIM_PeriodElapsedCallback
 *@function:���ö�ʱ����ˢ������,����ʱ����ֻ�в��޸�IRQHandler���ܴ����˺���
 *@param:��ʱ���ṹ��
 *@return:��
 **********************/
void MY_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        //���ڼ���������������;������ʱ�������¼������
        if (++update_count[1] >= 3) //�������ж��¼�����̫�࣬˵����ʱ�ĵ�����ڲ�ת��״̬���ʵ��ת����0
        {
            cap_cnt[1] = 0;
            cap_temp_val[1] = 0;
            status_flag[1] = 0;
            update_count[1] = 0;
            encoder_val[1] = 0;
        }
        if (++update_count[2] >= 3)
        {
            cap_cnt[2] = 0;
            cap_temp_val[2] = 0;
            status_flag[2] = 0;
            update_count[2] = 0;
            encoder_val[2] = 0;
        }
    }
    else if (htim->Instance == TIM5)
    {
        if (++update_count[3] >= 3)
        {
            cap_cnt[3] = 0;
            cap_temp_val[3] = 0;
            status_flag[3] = 0;
            update_count[3] = 0;
            encoder_val[3] = 0;
        }
        if (++update_count[4] >= 3)
        {
            cap_cnt[4] = 0;
            cap_temp_val[4] = 0;
            status_flag[4] = 0;
            update_count[4] = 0;
            encoder_val[4] = 0;
        }
    }
	else if(htim->Instance == TIM10)
	{
	if(__HAL_TIM_GET_IT_SOURCE(&htim10,TIM_IT_UPDATE)==SET)//????
    {
     sin1.time++;
    }
    __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);//???????
	
	
	}
}
/**********************************************************************
 * @Name    HAL_TIM_IC_CaptureCallback
 * @declaration : handle tim ic event for encoder
 * @param   htim: [����/��] tim structure ptr
 * @retval   : void
 * @author  peach99CPP
 ***********************************************************************/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    double temp_val = 0;
    if (htim == motor1.IC.Tim && htim->Channel == motor1.IC.Active_Channel)
    {
        if (!status_flag[1]) //��һ�β����ǲ���������
        {
            status_flag[1] = 1;                                                          //״̬��־λ��1���´ν��ж�������һ��
            rising_val[1] = HAL_TIM_ReadCapturedValue(motor1.IC.Tim, motor1.IC.Channel); //��ȡ��ʱ�����ص�ֵ
            update_count[1] = 0;                                                         //�����¼������� ��0
            //�жϷ��򣬷ֱ�����ת���Ƿ�ת
            if (HAL_GPIO_ReadPin(motor1.Encoder_IO.Port, motor1.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[1] = FORWARD;
            }
            else
            {
                direct_[1] = BACKWARD;
            }
            __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_FALLING); //��һ���ǲ����½���
        }
        else //�����½���
        {
            status_flag[1] = 0;                                                                                                 //״̬λ�����һ������ѭ����ɡ���һ�ξ��ǲ���������
            falling_val[1] = HAL_TIM_ReadCapturedValue(motor1.IC.Tim, motor1.IC.Channel);                                       //��ȡ�½��ص�ֵ
            cap_temp_val[1] += (SPEED_PARAM / (falling_val[1] - rising_val[1] + TIM_COUNT_VAL * update_count[1])) * direct_[1]; //���㱾�εõ���������ӳ��ת�ٵĿ��������ۼ�
            update_count[1] = 0;
            cap_cnt[1]++; //���������ۼӣ����ݲ����������˲��������ֵ

            __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_RISING); //������ѭ����ɣ��ص���ʼ״̬��׼���������ؽ��в���
            num_cnt[1]++;
            if (cap_cnt[1] == FILTER) //��������������
            {
                if (!first_flag[1]) //��һ�ε�ʱ����Ϊû����һ�ε�ֵ����Ҫ�������⴦��
                {
                    first_flag[1] = 1;
                    encoder_val[1] = cap_temp_val[1] / FILTER;
                }
                else
                {
                    //�ձ�����

                    temp_val = -cap_temp_val[1] / FILTER;                //��ȡ�����������ڵ�ƽ��ֵ
                    if (!(fabs(temp_val + encoder_val[1]) < THRESHOLD_)) //û����Ϊë�̷����������䣬�еĻ�ֱ���������λ�õ�ֵ
                    {
                        //��ֵ�˲�
                        temp_val += encoder_val[1];
                        encoder_val[1] = -temp_val / (2.0);
                        /*�������淽��ԭ������\
                        �и����ڵ�һ�ڶ���ִ�м�϶ʱ���ڣ�������ֵ����ȡ
                        ��ʱ��������ֵ��������ʱֵ���쳣�Ĵ�
                        ���������쳣
                        ��������
                        encoder_val[1] += temp_val;
                        encoder_val[1] /= 2.0;//��ֵ�˲�

                        */
                    }
                }
                //��ر�����0 ���ǵ���0��
                temp_val = 0;
                cap_cnt[1] = 0;
                cap_temp_val[1] = 0;
            }
        }
    }
    else if (htim == motor2.IC.Tim && htim->Channel == motor2.IC.Active_Channel)
    {

        if (!status_flag[2])
        {
            status_flag[2] = 1;
            rising_val[2] = HAL_TIM_ReadCapturedValue(motor2.IC.Tim, motor2.IC.Channel);
            update_count[2] = 0;

            if (HAL_GPIO_ReadPin(motor2.Encoder_IO.Port, motor2.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[2] = FORWARD;
            }
            else
            {
                direct_[2] = BACKWARD;
            }
            __HAL_TIM_SET_CAPTUREPOLARITY(motor2.IC.Tim, motor2.IC.Channel, TIM_ICPOLARITY_FALLING);
        }
        else
        {
            status_flag[2] = 0;
            falling_val[2] = HAL_TIM_ReadCapturedValue(motor2.IC.Tim, motor2.IC.Channel);
            cap_temp_val[2] += (SPEED_PARAM / (falling_val[2] - rising_val[2] + TIM_COUNT_VAL * update_count[2])) * direct_[2];
            update_count[2] = 0;
            cap_cnt[2]++;
            __HAL_TIM_SET_CAPTUREPOLARITY(motor2.IC.Tim, motor2.IC.Channel, TIM_ICPOLARITY_RISING);
            num_cnt[2]++;
            if (cap_cnt[2] == FILTER)
            {
                if (!first_flag[2])
                {
                    first_flag[2] = 1;
                    encoder_val[2] = cap_temp_val[2] / FILTER;
                }
                else
                {
                    temp_val = cap_temp_val[2] / FILTER;
                    if (!(fabs(temp_val + encoder_val[2]) < THRESHOLD_)) //û����Ϊë�̷�����������
                    {
                        temp_val += encoder_val[2];
                        encoder_val[2] = -temp_val / (2.0);
                    }
                }
                temp_val = 0;
                cap_cnt[2] = 0;
                cap_temp_val[2] = 0;
            }
        }
    }
    else if (htim == motor3.IC.Tim && htim->Channel == motor3.IC.Active_Channel)
    {

        if (!status_flag[3])
        {
            status_flag[3] = 1;
            rising_val[3] = HAL_TIM_ReadCapturedValue(motor3.IC.Tim, motor3.IC.Channel);
            update_count[3] = 0;
            if (HAL_GPIO_ReadPin(motor3.Encoder_IO.Port, motor3.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[3] = BACKWARD; // TODO �������޸���3�ŵ����ת�ٷ�����ж��߼�
            }
            else
            {
                direct_[3] = FORWARD;
            }
            __HAL_TIM_SET_CAPTUREPOLARITY(motor3.IC.Tim, motor3.IC.Channel, TIM_ICPOLARITY_FALLING);
        }
        else
        {
            status_flag[3] = 0;
            falling_val[3] = HAL_TIM_ReadCapturedValue(motor3.IC.Tim, motor3.IC.Channel);
            cap_temp_val[3] += (SPEED_PARAM / (falling_val[3] - rising_val[3] + TIM_COUNT_VAL * update_count[3])) * direct_[3];
            update_count[3] = 0;
            cap_cnt[3]++;
            __HAL_TIM_SET_CAPTUREPOLARITY(motor3.IC.Tim, motor3.IC.Channel, TIM_ICPOLARITY_RISING);
            num_cnt[3]++;
            if (cap_cnt[3] == FILTER)
            {
                if (!first_flag[3])
                {
                    first_flag[3] = 1;
                    encoder_val[3] = cap_temp_val[3] / FILTER;
                }
                else
                {
                    temp_val = cap_temp_val[3] / FILTER;
                    if (!(fabs(temp_val + encoder_val[3]) < THRESHOLD_)) //û����Ϊë�̷�����������
                    {
                        temp_val += encoder_val[3];
                        encoder_val[3] = -temp_val / (2.0);
                    }
                }
                temp_val = 0;
                cap_cnt[3] = 0;
                cap_temp_val[3] = 0;
            }
        }
    }
    else if (htim == motor4.IC.Tim && htim->Channel == motor4.IC.Active_Channel)
    {

        if (!status_flag[4])
        {
            status_flag[4] = 1;
            rising_val[4] = HAL_TIM_ReadCapturedValue(motor4.IC.Tim, motor4.IC.Channel);
            update_count[4] = 0;
            if (HAL_GPIO_ReadPin(motor4.Encoder_IO.Port, motor4.Encoder_IO.Pin) == GPIO_PIN_RESET)
            {
                direct_[4] = FORWARD;
            }
            else
            {
                direct_[4] = BACKWARD;
            }
            __HAL_TIM_SET_CAPTUREPOLARITY(motor4.IC.Tim, motor4.IC.Channel, TIM_ICPOLARITY_FALLING);
        }
        else
        {
            status_flag[4] = 0;
            falling_val[4] = HAL_TIM_ReadCapturedValue(motor4.IC.Tim, motor4.IC.Channel);
            cap_temp_val[4] += (SPEED_PARAM / (falling_val[4] - rising_val[4] + TIM_COUNT_VAL * update_count[4])) * direct_[4];
            update_count[4] = 0;
            cap_cnt[4]++;
            __HAL_TIM_SET_CAPTUREPOLARITY(motor4.IC.Tim, motor4.IC.Channel, TIM_ICPOLARITY_RISING);
            num_cnt[4]++;
            if (cap_cnt[4] == FILTER)
            {

                if (!first_flag[4])
                {
                    first_flag[4] = 1;
                    encoder_val[4] = cap_temp_val[4] / FILTER;
                }
                else
                {
                    temp_val = cap_temp_val[4] / FILTER;
                    if (!(fabs(temp_val + encoder_val[4]) < THRESHOLD_)) //û����Ϊë�̷�����������
                    {
                        temp_val += encoder_val[4];
                        encoder_val[4] = -temp_val / (2.0);
                    }
                }
                temp_val = 0;
                cap_cnt[4] = 0;
                cap_temp_val[4] = 0;
            }
        }
    }
    count_++;
    if (count_ == 400)
    {
        temp_sum = 0;
        count_ = 0;
        for (uint8_t t = 1; t <= 4; ++t)
        {
            temp_sum += fabs(num_cnt[t]);
            num_cnt[t] = 0;
        }
        encoder_sum += (temp_sum / 4.0);
    }
}

// static double pulse_queue[5][FILTER] = { 0 };
// static double queue_sum[5] = { 0 };
// static uint8_t queue_tail[5] = { 0 };
// static uint8_t queue_size[5] = { 0 };
// /***********************************************
//  * @brief �����������ֵ�˲�, ʹ�� FILTER �궨��
//  * @param motor_id ���id
//  * @param cap_pulse ���벶������
//  * @author TanES
//  ************************************************/
// double motor_filter(uint8_t motor_id, double cap_pulse)
// {
//     if (pulse_queue[motor_id][FILTER - 1] == 0) // ����δ��
//     {
//         pulse_queue[motor_id][queue_tail[motor_id]] = cap_pulse;
//         queue_tail[motor_id] = (queue_tail[motor_id] + 1) % FILTER;  // ���¶�βλ�ã�ʵ��ѭ������
//         queue_sum[motor_id] += cap_pulse;
//     }
//     else  // ��������
//     {
//         queue_sum[motor_id] -= pulse_queue[motor_id][queue_tail[motor_id]];  // ��ȥ��ɵ�����ֵ
//         pulse_queue[motor_id][queue_tail[motor_id]] = cap_pulse;  // �������µ�����ֵ
//         queue_sum[motor_id] += cap_pulse;  // �������µ�����ֵ
//         queue_tail[motor_id] = (queue_tail[motor_id] + 1) % FILTER;  // ���¶�βλ��
//         return queue_sum[motor_id] / FILTER;  // �����˲����ֵ
//     }
//      return 0;  // �� if ��֧��û�з���ֵ���������һ��Ĭ�ϵķ���ֵ
// }
// /***********************************************
//  * @brief ��η��������ж���Ҫ����˲���
//  * @param motor_id 
//  ************************************************/
// void motor_filter_set_zero(uint8_t motor_id)
// {
// 	memset(pulse_queue[motor_id], 0, sizeof(pulse_queue[motor_id]));
// 	queue_sum[motor_id] = queue_tail[motor_id] = queue_size[motor_id] = 0;
// }
