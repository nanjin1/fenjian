#include "chassis.h"
#include "motor.h"
#define TIME_PARAM 10
#define CHASSIS_RADIUS 1.0
#define MAX_CHASSIS_SPEED 500
#define MAX_SPEED 120.0 //370.0
#define MAX_CONTROL_VAL 19500
#define go 1
#define radius1 12
#define radius2 65
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "HWT101_imu.h"
#include "chassis_control.h"
#include "sin.h"

int W_Change=290;
double x_error = 0, y_error = 0, w_error = 0, final_W_error = 0;
float bar_factor = 1 / 10;
int i;
double speed_factor = 0, min_val;
CHASSIS_t chassis;
bool y_decide, x_Leftdecide, x_Rightdecide;  //ȷ���Ƿ�ʮ���ϣ�x.y����
float F_Light = 0, L_Light = 0, R_Light = 0; //ͨ������С���������Ƕ�
float Radius_[5] = {0,
                    0.4,
                    0.4,
                    0.4,
                    0.4};
float motor_target[5];
short time_count;
extern pid_paramer_t motor_param;
extern pid_paramer_t motor_param1;
float control_val[5];
extern int ten_num; //Ҫ�߶��ٸ�ʮ��
extern int x_ten_status, y_ten_status;
int go_yuan_flag=0;
int yuan_in=0;
int yuan_out=0;
					
extern int LL_data[3];
void w_speed_set(float w_speed)
{
    chassis.w_speed = w_speed;
}

/**********************************************************************
 * @Name    get_chassis_speed
 * @declaration : get dir speed
 * @param   dir: [����/��] direct char
 * @retval   : specify direct speed
 * @author  peach99CPP
 ***********************************************************************/

float get_chassis_speed(char dir)
{
    if (dir == 'x' || dir == 'X')
    {
        return chassis.x_speed;
    }
    else if (dir == 'y' || dir == 'Y')
    {
        return chassis.y_speed;
    }
    else if (dir == 'w' || dir == 'W')
    {
        return chassis.w_speed;
    }
    else
        return 0;
}
/**********************************************************************
 * @Name    set_speed
 * @brief     ֱ���޸������̵��ٶ�ֵ
 * @param   x: [����/��]  x�����ٶ�
 **			 y: [����/��]  y�����ٶ�
 **			 w: [����/��]  w�����ٶ�
 * @retval  void
 * @author  peach99CPP
 * @Data    2021-08-06
 ***********************************************************************/
void set_speed(int x, int y, int w)
{
    if (chassis.enable_switch) //ֻ�е����̵�ʹ�ܿ��ر���ʱ��������в���
    {
        chassis.x_speed = x;
        chassis.y_speed = y;
        chassis.w_speed = w;
    }
}

/************************************************************
 *@name:set_chassis_status
 *@function:����ʹ�ܿ���
 *@param:״̬��boolֵ
 *@return:��
 **************************************************************/
void set_chassis_status(bool status)
{
    chassis.enable_switch = status;
}

/**********************************************************************
 * @Name    speed_variation
 * @brief  �����ṩ�ٶ��޸ĵĽӿ�
 * @param   x_var: [����/��] x�����ٶȵĸı���
 **			 y_var: [����/��] y�����ٶȵĸı���
 **			 w_var: [����/��] w�����ٶȵĸı���
 * @retval  void
 * @author  peach99CPP
 * @Data    2021-08-06
 ***********************************************************************/

void speed_variation(float x_var, float y_var, float w_var)
{
    if (chassis.enable_switch)
    {
        chassis.x_speed += x_var;
        chassis.y_speed += y_var;
        chassis.w_speed += w_var;
    }
}

void change_w_error(int change)
{
   W_Change=change;
  
}


/************************************************************
 *@name:chassis_synthetic_control
 *@function:���̵��ۺϿ��ƺ������������ֿ���
 *@param:��
 *@return:��
 **************************************************************/
void chassis_synthetic_control(void)
{

    static float x, y, w, factor;
    static double max_val;
    if (chassis.enable_switch == false)
        return; //������̲���ʹ�ܣ���û�к�������
    short fn;
    if (++time_count == TIME_PARAM)
    {
        time_count = 0;
        y_error = track_pid_cal(&y_bar);
        x_error = (track_pid_cal(&x_leftbar) - track_pid_cal(&x_rightbar)) / 2.0f;
    }
	
    w_error = imu_correct_val();
    if (w_error < 0)
    {
        fn = -1;
    }
    if (w_error > 0)
    {
        fn = 1;
    }
    // int MaxVal = W_Change + 0.8 * (fabs(chassis.x_speed) + fabs(chassis.y_speed));
    // if (fabs(w_error) > MaxVal)
    // {
    //     final_W_error = fn * MaxVal;
    // }
    // else
    // {
    //     final_W_error = w_error;
    // }
    final_W_error=w_error;
    y_error = x_error = 0;
    x = chassis.x_speed + y_error;
    y = chassis.y_speed - x_error;
    w = chassis.w_speed + final_W_error;
	
    max_val = 0; //�����ֵ���ݽ��г�ʼ��
    factor = 1;  //�������ӳ�ʼ��
    min_val = chassis.x_speed;
    if (min_val > y)
        min_val = chassis.y_speed;
    if (min_val > w)
        min_val = chassis.w_speed;
    if (min_val > 100)
    {
        speed_factor = min_val / 100.0;
    }
    else
        speed_factor = 1;
    //    if (debug_motor_id>0) //ѭ��״̬�£����������
    //    {
    //
    //            printf("%.2lf ,%.2f\r\n",read_encoder(debug_motor_id),motor_target[debug_motor_id] );
    //       //printf("%.2f  %.2f ", motor_data[debug_motor_id].feedback, motor_data[debug_motor_id].expect);
    //       //printf("\r\n");
    //
    //    }
    //�����ǿ���ʱ��Ѳ��ģʽ���Ѳ���ͨ��

    /***************************************
                1*************2
                 *************
                 *************
                 *************
                 *************
                3*************4
        ****************************************/
	
	
//		motor_target[1] =0.6*( y +  x - Radius_[1] * w);
//		motor_target[2] = (-y + x - Radius_[2] * w);
//		motor_target[3] = -0.6*(+ y - x - Radius_[3] * w);
//		motor_target[4] = 0.6*(- y -  x - Radius_[4] * w);


//���̽��㣬��ȥ�겻ͬ������̽���ĳ�O�ζ�

//		motor_target[1] =-(y -  x + Radius_[1] * w);
//		motor_target[2] =-(y + x + Radius_[2] * w);
//		motor_target[3] =(y - x - Radius_[3] * w);
//		motor_target[4] = (y +  x - Radius_[4] * w);

		motor_target[1] =-(y +  x + Radius_[1] * w);//??
		motor_target[4] =-(y + x - Radius_[2] * w);//??
		motor_target[2] =-(y - x - Radius_[3] * w);//??
		motor_target[3] =-(y -  x + Radius_[4] * w);//??

	
     //printf("%f      ,%f       ,%f        ,%f",motor_target[1],motor_target[2],motor_target[3],motor_target[4]);
    //����һ���޷����������ⵥ���ٶȹ��ߵ��¿���Ч��������
    max_val = 0;             //���ñȽϱ���
    for (i = 1; i <= 4; ++i) //�ҳ����ֵ
    {
        if (motor_target[i] > max_val)
            max_val = motor_target[i];
    }
    if (max_val > MAX_SPEED) //���ֵ�Ƿ����ƣ����в�����ȷ�����ֵ���ڷ�Χ����ת�ٱ��� ����
    {
        factor = MAX_SPEED / max_val;
        for (i = 1; i <= 4; ++i)
        {
            motor_target[i] *= factor;
        }
    }
    for (i = 1; i <= 4; ++i)
    {
        // TODO �����ҽ������޸� ��������������Ϊһ��
        /*
         *�Ե�����б���
         *���Ȼ�ȡת���ڴ�ֵ
         *��ȡ����������
         *����PID���㺯���õ�����ֵ,�Է���ֵ��ʽ����
         *�ɼ���ֵ���Ƶ��
         */
	
//		motor_target[i] =sin_generator(&sin1);
        motor_data[i].expect = motor_target[i];
        motor_data[i].feedback = read_encoder(i);
	//	int temp = pid_control(&motor_data[i], &motor_param);
        control_val[i] = pid_control(&motor_data[i], &motor_param);
		
		if (control_val[i] > MAX_CONTROL_VAL)
            control_val[i] = MAX_CONTROL_VAL;
        if (control_val[i] < -MAX_CONTROL_VAL)
            control_val[i] = -MAX_CONTROL_VAL;

//		if(debug_motor_id == i)
//		{
			set_motor(i, control_val[i]);
//		}
       // set_motor(i, control_val[i]);
	}
	show_speed();
    // printf("%f      ,%f       ,%f        ,%f        ,%f\r\n", fabs(read_encoder(1)), fabs(read_encoder(2)), fabs(read_encoder(3)), fabs(read_encoder(4)), motor_target[1]);
    // printf("%.2f\r\n", control_val[1]); // TODO �������
//    if (debug_motor_id != 0) //ѭ��״̬��
//    {
//        printf("%.2lf , %.2f,%.0f \r\n",
//               motor_data[debug_motor_id].feedback,
//               motor_data[debug_motor_id].expect,
//               control_val[debug_motor_id]);
//        // printf("%.2f  %.2f ", motor_data[debug_motor_id].feedback, motor_data[debug_motor_id].expect);
//        // printf("\r\n");
//    }
}
void show_speed1(void)
{
    printf("%.2lf  %.2f\r\n", motor_data[debug_motor_id].feedback, chassis.y_speed);
}
int Get_X_speed(void)
{
    return chassis.x_speed;
}
int Get_Y_speed(void)
{
    return chassis.y_speed;
}
int Get_W_speed(void)
{
    return chassis.w_speed;
}

void Set_Dir_Speed(char dir, int speed)
{
    switch (dir)
    {
    case 'x':
    case 'X':
        set_speed(speed, Get_Y_speed(), Get_W_speed());
        break;
    case 'y':
    case 'Y':
        set_speed(Get_X_speed(), speed, Get_W_speed());
        break;
    case 'w':
    case 'W':
        set_speed(Get_X_speed(), Get_Y_speed(), speed);
        break;
    default:;
    }
}

void Feng_Feng(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    osDelay(2000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}


void go_yuan(void)
{
   Set_IMUStatus(0);
	go_yuan_flag=1;

}




