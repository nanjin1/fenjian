

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
void Cmd_Convert(int cmd); //ת��ָ��

void Servo_Uart_Send(void);                                                    //ͨ�����ڷ���
void Single_Control(int id, int control_mode, int angle, int time, int delay); //���Ƶ������
                                    
int Get_Servo_Flag(void);                                                      //��ȡ������״̬
void Servo_RX_IRQ(void);                                                       //��ؽ����жϴ���
void Enable_ServoFlag(void);                                                   //ʹ��״̬λ
void Disable_ServoFlag(void);                                                  //���״̬λ

void Wait_Servo_Signal(long wait_time_num); //������ʱ���� �ȴ����ָ��
void Ass_Door(int status);                  //��ƨ�ɵ��Ž��򵹽�ȥ

bool Get_IFUP(void);        //��е���Ƿ��Ѿ�����
void Set_IFUP(bool status); //���û�е��״̬

uint8_t Get_Uint16_Transform(uint16_t obj, char type2trans);
void ActionGroup(uint8_t groupId, uint16_t run_times);
void ServoInfBack_IRQ(void);
void Servo_Rx_Deinit(void);
void Error_Report(int type);
extern ServoControler_t servo_controler;
extern uint8_t mv_rec_flag;
#endif
