/***
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 09:07:09
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-08 13:40:51
 * @Description: ����д���float
 */
#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include "time_cnt.h"

#define NULL 0

typedef struct pid_paramer
{
    //�����޷�ֵ
    float integrate_max;
    //���Ʋ���kp
    float kp;
    //���Ʋ���ki
    float ki;
    //���Ʋ���kd
    float kd;
    //����޷�
    float control_output_limit;
} pid_paramer_t;

typedef struct pid_data
{
    //����
    volatile float expect;
    //����ֵ
    float feedback;
	float lastfeedback;
    //ƫ��
    float err;
    //�ϴ�ƫ��
    float last2err;
    //���ϴ�   ����ʽר��
    float last_err;
    //����ֵ
    float integrate;
    //����ƫ��
    float delta;
    //ƫ��΢��
    float dis_err;
    //�����������
    float control_output;
    //���ʱ�����
    Testime pid_controller_dt;
    //˽������
    void *pri_data;
    //�Զ������ƫ�ƫ����ֻص�
    void (*err_callback)(struct pid_data *, struct pid_paramer *);
    //��·��־
    uint8_t short_circuit_flag;
} pid_data_t;

typedef void (*pid_callback_t)(struct pid_data *, struct pid_paramer *);

extern float Iout, Pout, Dout;
float imu_pos_pid_cal(pid_data_t *data, pid_paramer_t *para);
float delta_pid(pid_data_t *data, pid_paramer_t *para);
float pid_control(pid_data_t *data, pid_paramer_t *para);
float pos_pid_cal(pid_data_t *data, pid_paramer_t *para);
void pid_clear(pid_data_t *data);
#endif
