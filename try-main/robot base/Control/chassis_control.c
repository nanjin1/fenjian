#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "time_cnt.h"
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "HWT101_imu.h"
#include "motor.h"
#include "imu_pid.h"
#include "Wait_BackInf.h"
#include "uart_handle.h"
#include "stdbool.h"
#include "general_interface.h"
uint32_t time;

#define LINE_FACTOR 50

#define MAX_SPEED 120 //100
#define MIN_SPEED 40
#define LINE_ERROR_ENCODER 150

#define Min_boo1 0xC8
#define Max_boo1 0x1F4
#define Min_speed1 150

double encoder_factor = 10;
double en_dir, en_val;
int dir, lines, boo_num, boo_dir;
int count_line_status = 1, encodermove_status = 1, booo_status = 1;
int edge_status[3] = {0};
double bias = 0, variation;
bool home_back = false;
int GO_home = 0;

int far_flag = 0;
// short error;���ֲ������ĳ���ȫ�ֱ���
//  ����false ˵�����ǻؼ�״̬
bool Get_IfBackHome(void)
{
    return home_back;
}
//  1 ˵���ؼ�״̬
void Set_HomeBack(short if_home)
{
    if (if_home == 1)
    {
        home_back = true;
    }
    else
        home_back = false;
}
osThreadId Line_Handle = NULL;
void LineTask(void const *argument);

osThreadId Boo_Handle = NULL;
void BooTask(void const *argument);

osThreadId Encoder_Handle = NULL;
void EncoderTask(void const *argument);

void move_slantly(int dir, int speed, uint16_t delay)
{
    set_imu_status(true);
    track_status(1, 0);
    track_status(2, 0);
    int x_factor, y_factor;
    switch (dir)
    {
    case 1:
        x_factor = 1, y_factor = 1;
        break;
    case 2:
        x_factor = -1, y_factor = 1;
        break;
    case 3:
        x_factor = -1, y_factor = -1;
        break;
    case 4:
        x_factor = 1, y_factor = -1;
        break;
    default:
        x_factor = 0, y_factor = 0;
    }
    set_speed(x_factor * speed, y_factor * speed, 0);
    osDelay(delay);
    // todo ����ĸ�����Ҫ���н�һ���Ĳ�����ȷ������
    set_speed(0, 0, 0);   //ͣ��
    set_imu_status(true); //�ر�������
    track_status(1, 1);   //����ѭ������������̬
    track_status(2, 1);   //�����򶼿���
    osDelay(800);         //��һ����ʱ��
    Set_InitYaw(0);       //���ýǶ�
}

/**
 * @description:  ����һ��������������
 * @param {int} direct ��������Ϊ2 ����Ϊ1
 * @param {int} line_num ��������
 * @param {int} edge_if �Ƿ����ڱ�Ե������ֵ����������⴦��
 * @param {int} imu_if  �Ƿ���������
 * @return {*} ���践��ֵ
 */
void direct_move(int direct, int line_num, int imu_if, int edge_if)
{

    static int delay_time;
    if (count_line_status)
    {
    START_LINE:
        set_imu_status(imu_if);
        Clear_Line(&y_bar);
        if (edge_if)
            edge_status[0] = 1;
        else
            edge_status[0] = 0;

        count_line_status = 0;

        dir = direct;
        lines = line_num;
        osThreadDef(line_task, LineTask, osPriorityRealtime, 0, 256);
        Line_Handle = osThreadCreate(osThread(line_task), NULL);
    }
    else
    {
        //����һ���������������ʱ����ʱ�����ٶȵ����������񣬶��ǽ��еȴ���
        //���ȴ�ʱ�����ʱ��ֱ��ȡ�����������˶���
        delay_time = 0;
        while (!count_line_status)
        {
            delay_time++;
            osDelay(100);
            if (delay_time >= 200)
            {
                printf("\n��������ȴ�ʱ�����, �Ѿ��˳�\n");
                return;
            }
        }
        goto START_LINE;
    }
}
void LineTask(void const *argument)
{
    int if_need_zero = 0; //

    osDelay(10); //�л����� �����Ϣ
    while (1)
    {
        static int speed_set = 0;
        short error;
        y_bar.if_switch = true;
        // if_need_zero = 1;
        // Turn_angle(1, 180, 0);
        // lines *= -1;��y�Ḻ�����ʱ����
        if (lines > 0)
        {
            do
            {
                error = lines - y_bar.line_num;
                if (error == 0)
                {
                    printf("\n\t �������\n");
                    set_speed(0, MIN_SPEED, 0);
                    Comfirm_Online(3);
                    goto EXIT_TASK;
                }
                set_speed(0, -speed_set, 0);
                osDelay(5);
            } while (error >= 0);
        }
        if (lines < 0)
        {
            if_need_zero = 1;
            Turn_angle(1, 180, 0);
            lines *= -1;
            do
            {
                error = lines - y_bar.line_num;
                if (error == 0)
                {
                    printf("\n\t �������\n");
                    set_speed(0, MIN_SPEED, 0);
                    Comfirm_Online(3);
                    goto EXIT_TASK;
                }
                set_speed(0, speed_set, 0);
                osDelay(5);
            } while (error >= 0);
        }
    }
EXIT_TASK:

    set_speed(0, 0, 0);
    if (if_need_zero)
        Turn_angle(1, 180, 1);
    else
    {
        y_bar.if_switch = true;
        osDelay(1500);
        y_bar.if_switch = false;
    }
    count_line_status = 1;
    vTaskDelete(NULL);
    Line_Handle = NULL;

    // printf("%d",go_exit);
}
/**
 * @name: move_by_encoder
 * @brief: ���ڿ�����������������ĺ���
 * @param {int} direct ���� ��ֱΪ2 yԲ��  ˮƽΪ1 x��
 * @param {int} val ����ƽ������ϵ��XY��������Ҫ�ߵ�·��
 * @return {*} ��
 */
void move_by_encoder(int direct, int val)
{
    static int encoder_delay;
	
	if(val>10||val<-10)
    val = (double)(val / 10.0);
	else if(val>=-10&&val<0)
		val = -1;
	else if(val<=10&&val>0)
		val = 1;
		
    if (encodermove_status)
    {
    START_ENCODER:
        en_dir = direct;
        en_val = val;
        
        encoder_sum = 0;

        osThreadDef(encodermove, EncoderTask, osPriorityRealtime, 0, 256);
        Encoder_Handle = osThreadCreate(osThread(encodermove), NULL);

        encodermove_status = 0;
    }
    else
    {
        encoder_delay = 0;
        while (!encodermove_status)
        {
            encoder_delay++;
            osDelay(100);
            if (encoder_delay >= 20)
            {
                set_speed(0, 0, 0);
                return;
            }
        }
        goto START_ENCODER;
    }
}
void get_encoder_factor(double factor)
{
    encoder_factor = factor;
}
void EncoderTask(void const *argument)
{
#define ENOCDER_DIVIDE_FACTOR 1.0 / 50.0
#define ENCODE_THRESHOLD 0.5
    static int pn;
    clear_motor_data();
    time = TIME_ISR_CNT;
	
    if (en_val < 0)
    {
        pn = -1;
        en_val *= -1;
		
    }
    else
    {
        en_val += 1;
        pn = 1;
    }

    if (en_dir == 1)
    {
        if (!Get_IfBackHome())
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = true;
            x_rightbar.if_switch = true;
        }
        else
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
        }
        while (1)
        {
            if ((TIME_ISR_CNT - time > 10) && ((en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD))
                goto Encoder_Exit;
            bias = fabs(en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR));
            variation = bias * encoder_factor;
            variation = Limit_Speed(variation)*1.5;
            set_speed(80*pn, 0, 0);
            osDelay(5);
        }
    }
    else if (en_dir == 2)
    {

        if (!Get_IfBackHome())
        {
            y_bar.if_switch = true;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
        }
        else
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
        }

        while (1)
        {
			
			
			if(GO_home == 0)
			{
			
            if (TIME_ISR_CNT - time > 600)
                goto Encoder_Exit;
            if ((TIME_ISR_CNT - time > 10) && ((en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD))
                goto Encoder_Exit;				
            bias = fabs(en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR));
            variation = bias * encoder_factor;
            variation = Limit_Speed(variation)*1.5;
			
            set_speed(0, variation * pn, 0);
            osDelay(5);
			}
			else
			{
			
			if (TIME_ISR_CNT - time > 600)
                goto Encoder_Exit;
            if ((TIME_ISR_CNT - time > 10) && ((en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD))
                goto Encoder_Exit;				
            bias = fabs(en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR));
            variation = bias * encoder_factor;
            variation = Limit_Speed(variation)*1.5;
			
            set_speed(0, 150 * pn, 0);
            osDelay(5);
			
			
			}
            // TODO �����Ƿ��ڴ˴��ر�������
        }
    }
Encoder_Exit:
    set_speed(0, 0, 0);
    track_status(1, 0);
    track_status(2, 0);
    osDelay(800);
    encodermove_status = 1;
    vTaskDelete(NULL);
    Encoder_Handle = NULL;
}

void car_shaking(int direct)
{
    while (y_bar.num == 0)
    {
        w_speed_set(40);
        osDelay(100);
        if (y_bar.num != 0)
            break;
        w_speed_set(-40);
        osDelay(100);
    }
}

int get_count_line_status(void)
{
    return count_line_status;
}

int get_enocdermove_status(void)
{
    return encodermove_status;
}
int get_Boomove_status(void)
{
    return booo_status;
}
int Limit_Speed(int speed)
{
    if (speed > 0)
    {
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;
        if (speed < MIN_SPEED)
            speed = MIN_SPEED;
    }
    else
    {
        if (speed < -MAX_SPEED)
            speed = -MAX_SPEED;

        if (speed > -MIN_SPEED)
            speed = -MIN_SPEED;
    }
    return speed;
}
void Comfirm_Online(int dir)
{
#define LOW_SPEED_TO_CONFIRM 120
#define Track_Time 10
    Set_IMUStatus(1);
    track_status(1, 1);
    track_status(2, 1);
    if (dir == 1)
    {
        if (Get_Trcker_Num(&y_bar) <= 2)
        {
            set_speed(-LOW_SPEED_TO_CONFIRM, 0, 0);
            while (Get_Trcker_Num(&y_bar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    else if (dir == 2)
    {
        if (Get_Trcker_Num(&y_bar) <= 5)
        {
            set_speed(0, -LOW_SPEED_TO_CONFIRM, 0);
            while (Get_Trcker_Num(&y_bar) <= 5)
                osDelay(10);
            osDelay(1000);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    else if (dir == 3)
    {
        if (Get_Trcker_Num(&x_leftbar) <= 2 && Get_Trcker_Num(&x_rightbar) <= 2)
        {
            set_speed(0, LOW_SPEED_TO_CONFIRM, 0);
            while (Get_Trcker_Num(&x_leftbar) <= 2 && Get_Trcker_Num(&x_rightbar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    track_status(1, 0);
}
/**
 * @name: Wait_OKInf
 * @brief: ���ڻ�ȡ��һ�������Ƿ����н���
 * @param {int} type �жϵ�����
 * @param {long} wait_time ��ߵĵȵ�ʱ��
 * @return {*}
 */
long Wait_OKInf(int type, long wait_time)
{
    long temp_time = 5;
    if (type == 1)
    {
        osDelay(temp_time);
        while (!get_count_line_status() && temp_time < wait_time)
        {
            temp_time += 5;
            osDelay(5);
        }
        if (get_count_line_status())
        {
            goto EXIT_WAIT;
        }
    }
    else if (type == 2)
    {
        osDelay(temp_time);
        while (!get_enocdermove_status() && temp_time < wait_time)
        {
            temp_time += 5;
            osDelay(5);
        }
        if (get_enocdermove_status())
        {
            goto EXIT_WAIT;
        }
    }
    else if (type == 3)
    {
        osDelay(temp_time);
        while (!(get_Boomove_status()) && temp_time < wait_time)
        {
            temp_time += 5;
            osDelay(5);
        }
        if (get_Boomove_status())
        {
            goto EXIT_WAIT;
        }
    }
    osDelay(100);
EXIT_WAIT:
    return temp_time;
}

/**
 * @name:  inte_move
 * @brief: �˶��ļ���
 * @param {int} type  ���� 1 Ϊ���� 2Ϊ�������
 * @param {int} dir   ���� 1Ϊ���� 2Ϊ����
 * @param {int} val    ��  ��Ҫ����ֵ �ֱ��Ӧ���ߵ���ֵ�ͱ�������ֵ
 * @param {int} edge    ������ģʽ��ʹ�� ���������Ƿ�Ϊ��Ե״̬
 * @param {int} imu_if  �������Ƿ���������
 * @param {long} wait_time  ��ĵȴ�ʱ��
 * @return {*}              �����Ƿ����гɹ�  ������false�����д������
 */
bool inte_move(int type, int dir, int val, int edge, int imu_if, long wait_time)
{
    if (type == 1)
    {
        direct_move(dir, val, imu_if, edge);
        Wait_OKInf(type, wait_time);
        return true;
    }
    else if (type == 2)
    {
        move_by_encoder(dir, val);
        Wait_OKInf(type, wait_time);
        return true;
    }

    printf("����Ĳ������ͳ��� \n");
    return false;
}

void BOO_Begin(int direct, int num)
{
    static int delay_time;
    if (booo_status)
    {
    START_BOO:
        booo_status = 0;
        boo_dir = direct;
        boo_num = num;

        osThreadDef(Boo_task, BooTask, osPriorityRealtime, 0, 256);
        Boo_Handle = osThreadCreate(osThread(Boo_task), NULL);
    }
    else
    {
        delay_time = 0;
        while (!booo_status)
        {
            delay_time++;
            osDelay(100);
            if (delay_time >= 200)
            {
                printf("\n�������ȴ�ʱ�����, �Ѿ��˳�\n");
                return;
            }
        }
        goto START_BOO;
    }
}

void BooTask(void const *argument)
{
    osDelay(10);
    int far_num;
    while (1)
    {
        short error;

        // far_bong=Get_US();
        // printf("%d\r\n",far_bong);
        int speed = 0;
        if (boo_dir == 1)
        {
            do
            {

                far_num = Get_US_NUM();
                // printf("0xFF%d\r\n",far_num);
                error = boo_num - far_num;

                speed = error * Min_speed1;

                if (error == 0)
                {

                    printf("\n\t ������over over over!");

                    goto BOO_TASK;
                }
                set_speed(0, speed, 0);
                osDelay(5);

            } while (error >= 0);
        }
        if (boo_dir == -1)
        {
            do
            {

                far_num = Get_US_NUM();
                printf("%d\r\n", far_num);
                error = boo_num - far_num;

                speed = Min_speed1 * error;

                if (error == 0)
                {
                    printf("\n\t ������over over over!");

                    goto BOO_TASK;
                }
                set_speed(0, -speed, 0);
                osDelay(5);

            } while (error >= 0);
        }
        if (boo_dir == 2)
        {

            do
            {
                far_num = Get_US_NUM();
                printf("%d\r\n", far_num);
                error = boo_num - far_num;
                speed = Min_speed1 * error;
                if (error == 0)
                {
                    printf("\n\t ������over over over!");

                    goto BOO_TASK;
                }
                set_speed(speed, 0, 0);
                osDelay(5);

            } while (error >= 0);
        }
    }
BOO_TASK:
    set_speed(0, 0, 0);
    Clear_Boo(&Boo);
    booo_status = 1;
    vTaskDelete(NULL); //����ȡ�������ڽ�����ʱ��
    Boo_Handle = NULL;
}

void Circle_around(int x_speed,int y_speed,int w_speed)
{

	
	
	bool should_continue = true;
	while (should_continue)
	{
		set_speed(0, y_speed, 0);
		while (Get_HW_others(4))
		{
			
			set_speed(0, 0, 0);
			osDelay(1000);
			should_continue = false;
			break;
		}
	}
	should_continue = true;
	//������һ�δ�����y�ᶨλ����׮�Ĵ���
	
	
	while(should_continue)
	{
		set_speed(-50, 0, 0);
		while (Get_HW_others(3))
		{
			//HAL_Delay(0);
			
			set_speed(0, 0, 0);
			
			if(Get_HW_others(3))
			{
				osDelay(1000);
			should_continue = false;
			break;
			}
		}
	}
	should_continue = true;
	//������һ�δ�����x�ᶨλ����׮�Ĵ���
	
	Turn_angle(1,90,0);
	Set_InitYaw(0);
	set_imu_status(0);
	
	int flag = 1;
	while(should_continue)
	{
		if(flag==1)
		{
			set_speed(60,0,44);
			osDelay(3000);
			flag = 0;
		}
//		if(flag==2)
//		{
//			set_speed(60,0,44);
//			osDelay(3000);
//			flag = 0;
//		}
//		if(Get_Yaw()>=175&&Get_Yaw()<=180)
//			{
//			osDelay(5);
//			if(Get_Yaw()>=170&&Get_Yaw()<=180)
//			set_speed(0,0,0);
//			osDelay(2000);
//			flag = 2;
//		
//		}
		if(Get_Yaw()>=-5&&Get_Yaw()<=0)
		{
			osDelay(5);
			if(Get_Yaw()>=-6&&Get_Yaw()<=0)
				set_speed(0,0,0);
			break;
		}
	
	
	}
	
//	set_imu_status(1);
//	int x,w = 0;
//	x = 35;
//	w = 70;
//		should_continue = true;
//	while(should_continue)
//	{
//		if(Get_HW_others(3) == 1 && Get_HW_others(4) == 0)
//		{
//			x += 5;
//			set_speed(0, x, w);
//		}
//		
//		else if(Get_HW_others(3) == 0 && Get_HW_others(4) == 1)
//		{
//			x_speed -= 5;
//			set_speed(0, x, w);
//		}
//		
//		else if(Get_HW_others(3) == 1 && Get_HW_others(4) == 1)
//		{
//			set_speed(0, x, w);
//		}
//		
//		osDelay(100);
//	}
}


