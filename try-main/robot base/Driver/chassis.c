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
bool y_decide, x_Leftdecide, x_Rightdecide;  //确定是否到十字上，x.y方向
float F_Light = 0, L_Light = 0, R_Light = 0; //通过三个小激光修正角度
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
extern int ten_num; //要走多少个十字
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
 * @param   dir: [输入/出] direct char
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
 * @brief     直接修改器底盘的速度值
 * @param   x: [输入/出]  x方向速度
 **			 y: [输入/出]  y方向速度
 **			 w: [输入/出]  w方向速度
 * @retval  void
 * @author  peach99CPP
 * @Data    2021-08-06
 ***********************************************************************/
void set_speed(int x, int y, int w)
{
    if (chassis.enable_switch) //只有当底盘的使能开关被打开时才允许进行操作
    {
        chassis.x_speed = x;
        chassis.y_speed = y;
        chassis.w_speed = w;
    }
}

/************************************************************
 *@name:set_chassis_status
 *@function:底盘使能开关
 *@param:状态，bool值
 *@return:无
 **************************************************************/
void set_chassis_status(bool status)
{
    chassis.enable_switch = status;
}

/**********************************************************************
 * @Name    speed_variation
 * @brief  对外提供速度修改的接口
 * @param   x_var: [输入/出] x方向速度的改变量
 **			 y_var: [输入/出] y方向速度的改变量
 **			 w_var: [输入/出] w方向速度的改变量
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
 *@function:底盘的综合控制函数，包含多种控制
 *@param:无
 *@return:无
 **************************************************************/
void chassis_synthetic_control(void)
{

    static float x, y, w, factor;
    static double max_val;
    if (chassis.enable_switch == false)
        return; //如果底盘不被使能，则没有后续操作
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
	
    max_val = 0; //对最大值数据进行初始化
    factor = 1;  //倍率因子初始化
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
    //    if (debug_motor_id>0) //循迹状态下（用这个调）
    //    {
    //
    //            printf("%.2lf ,%.2f\r\n",read_encoder(debug_motor_id),motor_target[debug_motor_id] );
    //       //printf("%.2f  %.2f ", motor_data[debug_motor_id].feedback, motor_data[debug_motor_id].expect);
    //       //printf("\r\n");
    //
    //    }
    //陀螺仪开启时的巡线模式，已测试通过

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


//底盘解算，和去年不同今年底盘解算改成O形二

//		motor_target[1] =-(y -  x + Radius_[1] * w);
//		motor_target[2] =-(y + x + Radius_[2] * w);
//		motor_target[3] =(y - x - Radius_[3] * w);
//		motor_target[4] = (y +  x - Radius_[4] * w);

		motor_target[1] =-(y +  x + Radius_[1] * w);//??
		motor_target[4] =-(y + x - Radius_[2] * w);//??
		motor_target[2] =-(y - x - Radius_[3] * w);//??
		motor_target[3] =-(y -  x + Radius_[4] * w);//??

	
     //printf("%f      ,%f       ,%f        ,%f",motor_target[1],motor_target[2],motor_target[3],motor_target[4]);
    //再来一个限幅操作，避免单边速度过高导致控制效果不理想
    max_val = 0;             //重置比较变量
    for (i = 1; i <= 4; ++i) //找出最大值
    {
        if (motor_target[i] > max_val)
            max_val = motor_target[i];
    }
    if (max_val > MAX_SPEED) //最大值是否超限制，进行操作，确保最大值仍在范围内且转速比例 不变
    {
        factor = MAX_SPEED / max_val;
        for (i = 1; i <= 4; ++i)
        {
            motor_target[i] *= factor;
        }
    }
    for (i = 1; i <= 4; ++i)
    {
        // TODO 这里我进行了修改 将两个参数重置为一个
        /*
         *对电机进行遍历
         *首先获取转速期待值
         *读取编码器参数
         *传入PID计算函数得到计算值,以返回值形式传参
         *由计算值控制电机
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
    // printf("%.2f\r\n", control_val[1]); // TODO 调试输出
//    if (debug_motor_id != 0) //循迹状态下
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




