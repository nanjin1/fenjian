/***
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 09:07:09
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-08 13:40:51
 * @Description: 请填写简介float
 */
#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include "time_cnt.h"

#define NULL 0

typedef struct pid_paramer
{
    //积分限幅值
    float integrate_max;
    //控制参数kp
    float kp;
    //控制参数ki
    float ki;
    //控制参数kd
    float kd;
    //输出限幅
    float control_output_limit;
} pid_paramer_t;

typedef struct pid_data
{
    //期望
    volatile float expect;
    //反馈值
    float feedback;
	float lastfeedback;
    //偏差
    float err;
    //上次偏差
    float last2err;
    //上上次   增量式专用
    float last_err;
    //积分值
    float integrate;
    //单次偏差
    float delta;
    //偏差微分
    float dis_err;
    //控制器总输出
    float control_output;
    //间隔时间计算
    Testime pid_controller_dt;
    //私有数据
    void *pri_data;
    //自定义计算偏差，偏差积分回调
    void (*err_callback)(struct pid_data *, struct pid_paramer *);
    //短路标志
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
