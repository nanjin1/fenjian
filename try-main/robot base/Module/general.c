/*
 * @Author: rlantic
 * @Date: 2022-03-19 16:12:02
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-20 02:11:04
 * @FilePath: \MDK-ARMd:\robot\robot\Module\general.c
 * @Description:
 *
 * Copyright (c) 2022 by �û�/��˾��, All Rights Reserved.
 */
#include "general.h  "
#include "cmsis_os.h"

#include "delay.h"
#include "motor.h"
#include "chassis.h"
#include "imu_pid.h"
#include "track_bar_receive.h"
#include "chassis_control.h"
#include "openmv.h"
#include "servo.h"
#include "HWT101_imu.h"
#include "read_status.h"
#include "servo.h"
#include "general.h"
#include "avoid_obs.h"
//���ȴ���Ϣ�Ŀɶ��Ա���
#define Wait_Dealy_MAX 50000
#define Line_Type 1
#define Encoder_Type 2

int debug_function_id = 0;
int function_param = 2;

int target_color = 2;
/**
 * @name: Set_TargetColor
 * @brief: ����Ŀ�������ɫ
 * @param {int} target 1��2��
 * @return {*}
 */
void Set_TargetColor(int target)
{
    if (target == 1)
        printf("��ɫ\r\n");
    else if (target == 2)
        printf("��ɫ\r\n");
    else
    {
        printf("��������\r\n ��Ҫ�ش�����\r\n");
        return;
    }
    target_color = target;
}
/**
 * @name: Get_TargetColor
 * @brief: ��ȡĿ�����ɫ
 * @param {*} void
 * @return {*} 1 ��2�� �����ж�
 */
int Get_TargetColor(void)
{
    return target_color;
}
/**********************************************************************
 * @Name    Openmv_Scan_Bar
 * @declaration :开启OPENMV�?描条形平�?
 * @param   status: [输入/出]  开始还�?结�??
 **			 color: [输入/出] 要抓的�?�色，反馈给mv
 * @retval   : �?
 * @author  peach99CPP
 ***********************************************************************/
void Openmv_Scan_Bar(int status, int color)
{
    if (status == 1)
    {
        MV_Start();
        osDelay(100);
        MV_SendCmd(3, color);
        ActionGroup(5, 1);
    }
    else if (status == 0)
    {
        MV_Stop();
        ActionGroup(6, 1);
    }
}

// ���º�����Ϊ��ɫ�볡ʹ��
/**
 * @name: Home2Disc
 * @brief: �𲽵�Բ�̻� ����·��
 * @param {int} dir /���� ��1 ��2
 * @return {*}
 */
void Home2Disc(int dir)
{
    MV_Stop();                                //�ر����
    move_by_encoder(2, 25);                   //��ǰ�ߵ����� todo ��������
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ����
    //direct_move(2, 3, 0, 1);                  //�������� ֱ�ӵ���Ŀ����
    Wait_OKInf(Line_Type, Wait_Dealy_MAX * 10);
    Turn_angle(1, -90, 1);
    Comfirm_Online(3); // 4 15���޸� �ǵò���
}
void Home2DiscRed(int dir)
{
    MV_Stop();                                //�ر����
    move_by_encoder(2, 25);                   //��ǰ�ߵ����� todo ��������
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ����
    //direct_move(2, 3, 0, 1);                  //�������� ֱ�ӵ���Ŀ����
    Wait_OKInf(Line_Type, Wait_Dealy_MAX * 10);
    Turn_angle(1, 90, 1);
    Comfirm_Online(3); //���������� ֱ����ȷ�����ߺ���
}

void Disc2Plat(void)
{
    move_by_encoder(2, -4);
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    Turn_angle(1, -90, 1);
    //direct_move(1, 1, 1, 1);
    Wait_OKInf(Line_Type, Wait_Dealy_MAX);
    //direct_move(2, 1, 1, 1);
    Wait_OKInf(Line_Type, Wait_Dealy_MAX);
    move_by_encoder(2, 25);
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    Wait_Switches(3);
    // HWSwitch_Move(6, 1);
}
/**
 * @name: Disc2Ware2plat
 * @brief: Բ�̻����ֿ⵽����
 * @param {*}
 * @return {*}
 */
void Disc2Ware2plat(void)
{
//     // ����������� ��ͷһֱ��ֱ���ŵ� ����ת��
//     move_by_encoder(2, -4); //���� ׼��ת�� todo ����-4�����Ƿ���� ֮ǰ���Ե�-6ƫ��
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
//     // ������ todo 4.11���� Բ�̻زֿ�ķ���
//     Turn_angle(1, -90, 1);
//     direct_move(2, 1, 0, 1);
//     Wait_OKInf(Line_Type, Wait_Dealy_MAX);

//     move_by_encoder(2, 40);
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
//     Turn_angle(1, -90, 0);                    //��ƨ�ɶ�׼�ֿ�
//     Wait_Switches(4);                         //�ó�β��׼�ֿ⵲��
//     Lateral_infrared(1);                      //�򿪲���ĺ���
//     Set_IMUStatus(1);                         //ͬ����볡
//     Kiss_Ass(1, 1);                           //�ò����������׼λ��
//     Wait_Switches(4);                         //��ȫ��ʩ todo �������������
//     Ass_Door(1);                              //����һϵ�ж�����
//     Lateral_infrared(0);                      //�Ѳ���ĺ�������
//     move_by_encoder(2, 6);                    //ǰ�������� todo ����ȷ��
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ����Ϣ
//     direct_move(2, 1, 0, 1);                  //ǰ��������
//     Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//     Turn_angle(1, 90, 1); //ת��
//     Wait_Switches(3);     //��׼
//     HWSwitch_Move(6, 1);  //��׼λ��
//     // todo �������
// }
// void Disc2Ware2platRed(void)
// {
//     // ����������� ��ͷһֱ��ֱ���ŵ� ����ת��
//     move_by_encoder(2, -4); //���� ׼��ת�� todo ����-4�����Ƿ���� ֮ǰ���Ե�-6ƫ��
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
//     Turn_angle(1, 90, 1);    //ת�䳵ͷ��׼��
//     direct_move(2, 1, 0, 1); //�����ƶ��������ֿ��·��
//     Wait_OKInf(Line_Type, Wait_Dealy_MAX);

//     move_by_encoder(2, 40); //��ǰ�߶�һ�����ܿ�ľ���϶
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
//     //  �˴�����ʹ���˵����ĳ��� todo ����Ϊ��������
//     Turn_angle(1, 90, 0); //��ƨ�ɶ�׼�ֿ�
//     Wait_Switches(4);     //�ó�β��׼�ֿ⵲��
//     Lateral_infrared(1);  //�򿪲���ĺ���
//     Set_IMUStatus(0);
//     Kiss_Ass(1, 1);      //�ò����������׼λ��
//     Wait_Switches(4);    //��ȫ��ʩ todo �������������
//     Ass_Door(1);         //����һϵ�ж�����
//     Lateral_infrared(0); //�Ѳ���ĺ�������
//     Set_IMUStatus(1);
//     move_by_encoder(2, 6);                    //ǰ�������� todo ����ȷ��
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ����Ϣ
//     direct_move(2, 1, 0, 1);                  //ǰ��������
//     Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//     Turn_angle(1, 90, 1); //ת��
//     Wait_Switches(3);     //��׼
//     HWSwitch_Move(6, 1);  //��׼λ��
//     // todo �������
}
/**
 * @name: plat2ware
 * @brief: �ӽ���ƽ̨��������
 * @param {*}
 * @return {*}
 */
void plat2ware2home(void)
{
    // //��ʱ���������ƽ̨�� ���ڽ��ݵ�5�ű�Ե
    // Action_Gruop(4, 1);                //��ȫ��ʩ
    // Wait_Servo_Signal(Wait_Dealy_MAX); //ȷ����һ�������
    // move_slantly(3, 120, 1500);        //б�ź��˵�����
    // Turn_angle(1, 90, 1);
    // Wait_Switches(4);                      //�ı��׼����
    // direct_move(2, 2, 1, 1);               //��ǰ�ߵ����� �����һ��·��
    // Wait_OKInf(Line_Type, Wait_Dealy_MAX); //�ȴ����

    // Wait_Switches(1);                         //ײ��ȥ ͬʱ�����Ƕ�
    // HWSwitch_Move(2, 1);                      //�����ƶ���ȷ����Ե
    // move_by_encoder(2, -4);                   //����һ��� ������������
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ�����
    // osDelay(300);
    // move_by_encoder(1, -6);                   //�ƶ���ָ��λ��׼���� todo ���Խ��䵹��B�� �����·���
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ�����
    // Wait_Switches(1);                         //��ֹ����һ���������뵲��
    // Action_Gruop(20, 1);                      //��������
    // Wait_Servo_Signal(Wait_Dealy_MAX);        //ȷ����һ�������
    // move_by_encoder(2, -6);                   //���������
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ�����
    // Turn_angle(1, 90, 1);                     //ת90�� ��ʱ��ͷ����Բ�̻�
    // move_by_encoder(2, -4);                   //�ܿ�ľ��ķ�϶  todo  4.15������С���ߴ� �����Ծ������
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ�����
    // Set_SwitchSpeed(120);                     //��ʱ���ø�һ��
    // Wait_Switches(3);                         //����ײ����������������̬
    // osDelay(200);                             //��һ�������¿�һ��
    // Set_SwitchSpeed(80);                      //�ָ���ʼ���ٶ�����
    // Ring_Move();                              //�ƶ���ȥ��Բ��
    // move_by_encoder(1, -4);                   //�ƶ�����  todo 412�޸Ĳ���
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ�����
    // Turn_angle(1, 180, 0);                    //ת��׼���ؼ�
    // Comfirm_Online(3);
    // Comfirm_Online(1);
    // Avoid_ObsOnwayBack();
    // set_speed(0, 0, 0); //ͣ��
    // Set_InitYaw(0);     //�Դ�ʱ�ĽǶ�Ϊ�趨�Ƕ�
    // Set_IMUStatus(0);   //�ر�������
}
void plat2ware2homeRed(void)
{
    // //��ʱ���������ƽ̨�� ���ڽ��ݵ�5�ű�ԵAction_Gruop(4,1);
    // Action_Gruop(4, 1); //��ȫ��ʩ
    // Wait_Servo_Signal(Wait_Dealy_MAX);
    // move_slantly(3, 120, 1700); //б�ź��˵�����
    // Turn_angle(1, 90, 1);
    // direct_move(2, 1, 1, 1);               //��ǰ�ߵ����� �����һ��·��
    // Wait_OKInf(Line_Type, Wait_Dealy_MAX); //�ȴ����

    // Wait_Switches(1);                         //ײ��ȥ ͬʱ�����Ƕ�
    // HWSwitch_Move(2, 1);                      //�����ƶ���ȷ����Ե
    // move_by_encoder(1, -9);                   //�ƶ���ָ��λ��׼���� todo 4��10������Ϊ9
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ�����
    // Wait_Switches(1);                         //��ֹ����һ���������뵲��
    // Action_Gruop(20, 1);                      //��������
    // Wait_Servo_Signal(Wait_Dealy_MAX);        //ȷ����һ�������
    // move_by_encoder(2, -6);                   //���������
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ�����
    // Turn_angle(1, 90, 1);                     //ת90�� ��ʱ��ͷ����Բ�̻�
    // move_by_encoder(2, -4);                   //�ܿ�ľ��ķ�϶
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //ȷ�����
    // Set_SwitchSpeed(120);                     //��ʱ���ø�һ��
    // Wait_Switches(3);                         //����ײ����������������̬
    // osDelay(200);                             //��һ�������¿�һ��
    // Set_SwitchSpeed(80);                      //�ָ���ʼ���ٶ�����
    // Ring_Move();                              //�ƶ���ȥ��Բ��
    // move_by_encoder(1, -4);                   //�ƶ�����  todo ��������ͨ��
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ�����
    // osDelay(100);                             //ֹͣһС��
    // Comfirm_Online(3);
    // Comfirm_Online(2);
    // osDelay(500);
    // Avoid_ObsOnwayBack(); //�ؼ�·��ʵ�ֱ���
    // set_speed(0, 0, 0);   //ͣ��
    // Set_InitYaw(0);       //�Դ�ʱ�ĽǶ�Ϊ�趨�Ƕ�
    // Set_IMUStatus(0);     //�ر�������
}
/**
 * @name: Set_Debug_Param
 * @brief: ���ó�������
 * @param {int} param  1 ��2��
 * @return {*}
 */
void Set_Debug_Param(int param)
{
    // function_param = param;
    // printf("\n������������Գ���\n");
}
/**
 * @name: Set_Debug_Task
 * @brief: ͨ����������������
 * @param {int} id 1��Բ�̻����� 2��Բ�̻��زֿ� 3�ӽ��ݻؼ�
 * @return {*}
 */
void Set_Debug_Task(int id)
{
    debug_function_id = id;
    WholeRun();
}
osThreadId WholeGameHandle = NULL;            //������
void WholeGameTaskFunc(void const *argument); //����ʵ�ֺ���
bool WholeTaskTask_Exit = 1;                  //�Ƿ��˳�

osThreadId CheckQrUpdateHandle = NULL;       //������
void QrUpdateTaskFunc(void const *argument); //����ʵ�ֺ���
bool QrkTask_Exit = 1;                       //�Ƿ��˳�
#define UpdateThreshold 5000

void StartCheckQrUpadte(void)
{
    if (CheckQrUpdateHandle == NULL)
    {
        osDelay(10);
        WholeTaskTask_Exit = 0;
        osThreadDef(QrUpdate, QrUpdateTaskFunc, osPriorityNormal, 0, 256); //��������ṹ��
        WholeGameHandle = osThreadCreate(osThread(QrUpdate), NULL);        //��������
    }
}
void Exit_QRcheck(void)
{
    QrkTask_Exit = 1;
}
void QrUpdateTaskFunc(void const *argument)
{
     static long coount_time = 0;
     static int current_val = 0;
     while (!QrkTask_Exit)
     {
     recount_symbol:
         current_val = Get_UpdateQr();
         while (Get_UpdateQr() == current_val && !QrkTask_Exit)
         {
             osDelay(5);
             coount_time += 5;
             if (coount_time >= UpdateThreshold)
             {
                 printf("\n\t����QR��־λ\n");
                 Set_temp_var(false);
                 Set_QrCount(0);
             }
         }
         osDelay(100);
         goto recount_symbol;
     }
     CheckQrUpdateHandle = NULL;
     vTaskDelete(NULL);
}
void WholeRun(void)
{
    if (WholeGameHandle == NULL)
    {
        osDelay(10);
        WholeTaskTask_Exit = 0;
        osThreadDef(WholeGame, WholeGameTaskFunc, osPriorityNormal, 0, 1024); //��������ṹ��
        WholeGameHandle = osThreadCreate(osThread(WholeGame), NULL);          //��������
    }
}

void WholeGameTaskFunc(void const *argument)
{
    // if (debug_function_id >= 0 && debug_function_id <= 3)
    // {
    //     Set_TargetColor(2);
    // }
    // else if (debug_function_id >= 5)
    // {
    //     Set_TargetColor(1);
    // }
    // switch (debug_function_id)
    // {
    // case 0:
    //     Set_TargetColor(2); //��ɫ
    //     Home2Disc(function_param);
    //     printf("����Բ�� ��ʼִ��Բ�̻�����\n");
    //     Disc_Mea();
    //     osDelay(100);
    //     printf("Բ�̽��� �زֿ��ȥ����\n");
    //     // Disc2Ware2plat();
    //     Disc2Ware2plat();
    //     osDelay(100);
    //     printf("��ʼ���н���\n");
    //     Brick_QR_Mode(5, 1, 1, 1);
    //     osDelay(100);
    //     printf("�ӽ��ݻؼ�\n");
    //     plat2ware2home();
    //     // plat2ware2home();
    //     break;
    // case 1:
    //     Home2Disc(function_param);
    //     break;
    // case 2:
    //     Disc2Ware2plat();
    //     break;
    // case 3:
    //     plat2ware2home();
    //     break;
    // case 4:
    //     Disc_Mea();
    //     break;
    // case 5:
    //     Home2DiscRed(function_param);
    //     break;
    // case 6:
    //     Disc2Ware2platRed();
    //     break;
    // case 7:
    //     plat2ware2homeRed();
    //     break;
    // case 8:
    //     Set_TargetColor(1); //��ɫ
    //     Home2DiscRed(function_param);
    //     printf("����Բ�� ��ʼִ��Բ�̻�����\n");
    //     Disc_Mea();
    //     printf("Բ�̽��� �زֿ��ȥ����\n");
    //     Disc2Ware2platRed();
    //     Brick_QR_Mode(5, 1, 1, 1);
    //     printf("�ӽ��ݻؼ�\n\n");
    //     plat2ware2homeRed();
    //     break;
    // case 9:;
    //     break;
    // case 10:;
    //     break;
    // case 11:
    //     Avoid_ObsOnwayBack();
    //     break;
    // default:
    //     printf("��������\n");
    //     break;
    // }
    // printf("���Ժ����������\n");
    // osDelay(200);
    // WholeGameHandle = NULL;
    // vTaskDelete(NULL);
}

#define Distance_Threshold 0XDB
void Avoid_ObsOnwayBack(void)
{
// #define Blue_Left 1
// #define Blue_Right 2
// #define Red_Left 3
// #define Red_Right 4
//     printf("\n\t������\n");
//     int avoid_dir = function_param;
//     if (avoid_dir == Blue_Left)
//     {
//         direct_move(1, 1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         while (Get_Distance() > Distance_Threshold)
//         {
//             set_speed(0, 120, 0);
//         }
//         osDelay(100);
//         direct_move(1, -1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         direct_move(2, 1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         Turn_angle(1, 180, 1);
//         Comfirm_Online(2);
//         //����ȷ������
//         Set_HomeBack(1);
//         //���ùر�ѭ��
//         move_by_encoder(1, -28); // todo
//         //ͨ���������ص���������ǰ
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ����
//         move_by_encoder(2, -14);                  // todo
//         //  ��ǰ�����������
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //���
//         track_status(1, 0);
//         track_status(2, 0);
//         Lateral_infrared(0); //ȷ������ĺ��ⱻ����
//     }
//     else if (avoid_dir == Blue_Right)
//     {
//         set_speed(0, 120, 0);
//         osDelay(600); //ˣ��
//         while (Get_Distance() > Distance_Threshold)
//         {
//             set_speed(0, 120, 0);
//         }
//         osDelay(100);
//         direct_move(1, 1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         Comfirm_Online(3);
//         Turn_angle(1, 180, 0);
//         Comfirm_Online(3);      //��������
//         move_by_encoder(1, 29); //�����²���
//         //ͨ���������ص���������ǰ
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ����
//         move_by_encoder(2, -15);
//         //  ��ǰ�����������
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //���
//         track_status(1, 0);
//         track_status(2, 0);
//         Lateral_infrared(0); //ȷ������ĺ��ⱻ����
//     }
//     else if (avoid_dir == Red_Left)
//     {
//         set_speed(0, 120, 0);
//         osDelay(500); //ˣ��
//         while (Get_Distance() > Distance_Threshold)
//         {
//             set_speed(0, 120, 0);
//         }
//         set_speed(0, 0, 0);
//         direct_move(1, -1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         Comfirm_Online(3);
//         Turn_angle(1, 180, 1);
//         Comfirm_Online(1);                        //ȷ������
//         Set_HomeBack(1);                          //���ô�ʱ�ǻؼ�״̬
//         move_by_encoder(1, -29);                  //��ʱ�������Ҳ�
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ����
//         move_by_encoder(2, -16);                  //  ��ǰ�����������
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //���
//     }
//     else if (avoid_dir == Red_Right)
//     {
//         direct_move(1, -1, 0, 1); //�����ƶ�������
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         while (Get_Distance() > Distance_Threshold)
//         {
//             set_speed(0, 120, 0);
//         }
//         set_speed(0, 0, 0);
//         direct_move(1, 1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         Comfirm_Online(3);
//         Turn_angle(1, 180, 1);
//         Comfirm_Online(1);                        //ȷ������
//         Set_HomeBack(1);                          //���ô�ʱ�ǻؼ�״̬
//         move_by_encoder(1, 29);                   //��ʱ�������Ҳ�
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //�ȴ����
//         move_by_encoder(2, -16);                  //  ��ǰ�����������
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //���
//     }
}

