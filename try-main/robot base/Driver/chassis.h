/*
 * @Author: rlantic
 * @Date: 2022-03-14 15:44:09
 * @LastEditors: OBKoro1
 * @LastEditTime: 2022-04-12 17:21:32
 * @FilePath: \MDK-ARMd:\program\motor\motor\Driver\chassis.h
 * @Description: 
 * 
 * Copyright (c) 2022 by �û�/��˾��, All Rights Reserved. 
 */


#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "main.h"
#include <stdbool.h>
#include "motor.h"
#include "tim_control.h"
typedef struct chassis_structure
{
    bool enable_switch;//�Ƿ�ʹ�ܵ���
    double x_speed;//x��������ٶ�
    double  y_speed;//y��������ٶ�
    double  w_speed;//w������̽��ٶ�
} CHASSIS_t;





extern CHASSIS_t chassis;


void w_speed_set(float w_speed);
void speed_variation(float x_var, float y_var, float w_var);
void set_speed (int x, int y, int w);
void set_chassis_status(bool status);
void chassis_synthetic_control(void);


int Get_X_speed(void);
int Get_Y_speed(void);
int Get_W_speed(void);

void Set_Dir_Speed(char dir, int speed);



float get_chassis_speed(char dir);

bool inte_move(int type, int dir, int val, int edge, int imu_if, long wait_time);//�˶����ۺ�

extern float motor_target[5];
extern float control_val[5];
extern int x_ten_status,y_ten_status;

bool Get_IfBackHome(void);

void Set_HomeBack(short if_home);
void contril_new(void);
void into_motor(void);
void all_ing(void);
void show_speed1(void);
void decide_Light(int num);
void Feng_Feng(void);
void change_w_error(int change);
void go_yuan(void);
#endif
