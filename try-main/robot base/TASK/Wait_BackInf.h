/************************************************************************
  *      
  * FileName   : Wait_BackInf.h   
  * Version    : v1.0		
  * Author     : 桃子			
  * Date       : 2021-10-14         
  * Description:    
*******************************************************************************/

#ifndef __WAIT__BACK_INF_H_
#define __WAIT__BACK_INF_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <stdbool.h>

bool Get_TimeResult(void);             //��ȡ��ʱ���
void Start_CountTime(long timelength); //��ʼ��ʱ
void Exit_CountTime(void);                 //ǿ���˳���ʱ��

int Get_CountTimeExit(void);
#endif
