/*
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 09:07:09
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-09 13:35:04
 * @Description: ����д���
 */
#include "pid.h"
#include "math.h"
#define ABS(X) (((X) > 0) ? (X) : -(X))
float Iout, Pout, Dout;
/**********************************************************************************************************
 *�� �� ��: pid_control
 *����˵��: pid����������
 *��    ��: pid���������ݽṹ�� pid����������
 *�� �� ֵ: �����
 **********************************************************************************************************/
float pid_control(pid_data_t *data, pid_paramer_t *para)
{
	if (data->expect==0)
	{
		data->control_output=0;
		return data->control_output;
	}
    float controller_dt;
    //��·ֱ������ڴ�ֵ
    if (data->short_circuit_flag)
    {
        data->control_output = data->expect;
        return data->control_output;
    }
    //��ȡdt
    Get_Time_Period(&data->pid_controller_dt);
    controller_dt = data->pid_controller_dt.Time_Delta / 1000000.0;
    //��һ�μ�����ʱ�佫���ּ��ʱ��ܴ�����
    if (controller_dt < 0.001f)
        return 0;
    //�����ϴ�ƫ��
    data->last_err = data->err;
    //������ȥ�����õ�ƫ��
    data->err = data->expect - data->feedback;
    //����ƫ��΢��
    //data->dis_err = data->err - data->last_err;
	data->dis_err = data->lastfeedback - data->feedback;
	data->lastfeedback = data->feedback;
    //�Զ���ƫ��΢�ִ���
//    if (data->err_callback)
//        data->err_callback(data, para);
    //�����޷�
    if (para->integrate_max)
    {
        if (data->integrate >= para->integrate_max)
            data->integrate = para->integrate_max;
        if (data->integrate <= -para->integrate_max)
            data->integrate = -para->integrate_max;
    }
    data->integrate += para->ki * data->err * controller_dt;
    //���������
    data->control_output = para->kp * data->err + data->integrate + para->kd * data->dis_err;
    //������޷�
    if (para->control_output_limit)
    {
        if (data->control_output >= para->control_output_limit)
            data->control_output = para->control_output_limit;
        if (data->control_output <= -para->control_output_limit)
            data->control_output = -para->control_output_limit;
    }
    //���������
    return data->control_output;
}
float delta_pid(pid_data_t *data, pid_paramer_t *para)
{
    float increasement = 0;
    data->last2err = data->last_err;
    data->last_err = data->err;
    data->err = data->expect - data->feedback;
	
    data->integrate = data->err * para->ki;
    if (para->integrate_max)
    {
        if (data->integrate >= para->integrate_max)
            data->integrate = para->integrate_max;
        if (data->integrate <= -para->integrate_max)
            data->integrate = -para->integrate_max;
    }
    increasement = para->kp * (data->err - data->last_err) + data->integrate+para->kd * (data->err - 2 * data->last_err + data->last2err);
    data->control_output = increasement + data->control_output;
    if (para->control_output_limit)
    {
        if (data->control_output >= para->control_output_limit)
            data->control_output = para->control_output_limit;
        if (data->control_output <= -para->control_output_limit)
            data->control_output = -para->control_output_limit;
    }
    return data->control_output;
}
float pos_pid_cal(pid_data_t *data, pid_paramer_t *para)
{
    data->last_err = data->err;
    data->err = data->expect - data->feedback;
	//if(data->err < 0.3)data->err = 0;
    data->integrate += data->err * para->ki;
    //�����޷�
    if (para->integrate_max)
    {
        if (data->integrate >= para->integrate_max)
            data->integrate = para->integrate_max;
        if (data->integrate <= -para->integrate_max)
            data->integrate = -para->integrate_max;
    }
    data->control_output = para->kp * data->err + data->integrate + para->kd * (data->err - data->last_err);
    if (para->control_output_limit)
    {
        if (data->control_output >= para->control_output_limit)
            data->control_output = para->control_output_limit;
        if (data->control_output <= -para->control_output_limit)
            data->control_output = -para->control_output_limit;
    }
    return data->control_output;
}
//���PID���ֵ��API
void pid_clear(pid_data_t *data)
{
    data->control_output = 0;
    data->integrate = 0;
    data->err = data->dis_err = data->last_err = 0;
}

float imu_pos_pid_cal(pid_data_t *data, pid_paramer_t *para)
{
    data->last_err = data->err;
    data->err = data->expect - data->feedback;
    if (fabs(data->err) < 0.5)
        return 0;
    data->integrate += data->err * para->ki;
    //�����޷�
    if (para->integrate_max)
    {
        if (data->integrate >= para->integrate_max)
            data->integrate = para->integrate_max;
        if (data->integrate <= -para->integrate_max)
            data->integrate = -para->integrate_max;
    }
    data->control_output = para->kp * data->err + data->integrate + para->kd * (data->err - data->last_err);
    if (para->control_output_limit)
    {
        if (data->control_output >= para->control_output_limit)
            data->control_output = para->control_output_limit;
        if (data->control_output <= -para->control_output_limit)
            data->control_output = -para->control_output_limit;
    }
    return data->control_output;
}
