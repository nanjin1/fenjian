
#ifndef _TIME_CNT_H
#define _TIME_CNT_H

#include "time_cnt.h"
#include <stdint.h>
#include "main.h"
typedef struct
{
    uint8_t inited;
    //��λus
    uint32_t Last_Time;
    uint32_t Now_Time;
    uint32_t Time_Delta;
} Testime;

typedef struct
{
    uint16_t hour;
    uint16_t minute;
    uint16_t second;
    uint16_t microsecond;
} Time_t;

struct BUZZER
{
    uint8_t flag;
    uint16_t time;
};
extern struct BUZZER buzzer;
extern Time_t Time_Sys;
extern volatile uint32_t TIME_ISR_CNT;
void T6_IRQHandler(TIM_HandleTypeDef *htim);
void Get_Time_Init(void);
void Get_Time_Period(Testime *Time_Lab);
int Get_second(void);
int Get_minute(void);
#endif
