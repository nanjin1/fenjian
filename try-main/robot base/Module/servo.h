

#ifndef __SERVO_H_
#define __SERVO_H_

#include <string.h>

#include "main.h"
#include "usart.h"
#include <stdbool.h>

#define MAX_SERVO_SIZE 100
#define MAX_SERVO_REC_SIZE 20
typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t current_index;
    uint8_t cmd_buffer[MAX_SERVO_SIZE];
    uint8_t rec_buffer[MAX_SERVO_REC_SIZE];
    uint8_t rec_index;
} ServoControler_t;

typedef enum
{
    Lowest = 12,
    Medium = 32,
    Highest = 22,
    Bar,
} Servo_ID_t;
void Cmd_Convert(int cmd); //转换指令

void Servo_Uart_Send(void);                                                    //通过串口发送
void Single_Control(int id, int control_mode, int angle, int time, int delay); //控制单个舵机
                                    
int Get_Servo_Flag(void);                                                      //获取舵控完成状态
void Servo_RX_IRQ(void);                                                       //舵控接收中断处理
void Enable_ServoFlag(void);                                                   //使能状态位
void Disable_ServoFlag(void);                                                  //清除状态位

void Wait_Servo_Signal(long wait_time_num); //创建超时任务 等待舵控指令
void Ass_Door(int status);                  //打开屁股的门将球倒进去

bool Get_IFUP(void);        //机械臂是否已经升起
void Set_IFUP(bool status); //设置机械臂状态

uint8_t Get_Uint16_Transform(uint16_t obj, char type2trans);
void ActionGroup(uint8_t groupId, uint16_t run_times);
void ServoInfBack_IRQ(void);
void Servo_Rx_Deinit(void);
void Error_Report(int type);
extern ServoControler_t servo_controler;
extern uint8_t mv_rec_flag;
#endif
