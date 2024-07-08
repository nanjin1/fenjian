/************************************************************************
  *
  * FileName   : chassis_control.h
  * Version    : v1.0
  * Author     : 桃子
  * Date       : 2021-08-06
  * Description:
  * Function List:
  	1. ....
  	   <version>:
  <modify staff>:
  		  <data>:
   <description>:
  	2. ...
*******************************************************************************/



#ifndef __CHASSIS_CONTROL_H_
#define __CHASSIS_CONTROL_H_
#include "math.h"

#define ABS(x) ( (x)>0?(x):-(x) )
typedef enum
{
    hor = 0, //horizontal
    ver //vertically
} direct_t;

#include "tim_control.h"

void edge_move(int direct, int line_num);
void move_by_encoder(int direct, int val);
void move_slantly(int dir,int speed,uint16_t delay);
void direct_move(int direct, int line_num, int imu_if,int edge_if);
void car_shaking(int direct);

int get_count_line_status(void);
int get_enocdermove_status(void);

int Limit_Speed(int speed); 
void Comfirm_Online(int dir);
void get_encoder_factor(double factor);
extern int edge_status[3];
void BOO_Begin(int direct,int boo_num);
int get_Boomove_status(void);
long Wait_OKInf(int type, long wait_time);
void Circle_around(int x_speed,int y_speed,int w_speed);

#endif




