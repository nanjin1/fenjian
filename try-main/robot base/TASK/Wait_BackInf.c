#include "Wait_BackInf.h "
#include "uart_handle.h"
long wait_time = 0, now_time; //��ʱ��
bool OverTimeFlag = 1;        //���ʱ�ı�־

osThreadId OverTimeHandle = NULL;             //������
void CountTimeTaskFunc(void const *argument); //����ʵ�ֺ���
bool CountTimeTask_Exit = 1;                  //�Ƿ��˳�

/**********************************************************************
 * @Name    Get_TimeResult
 * @declaration :��ȡ��ʱ���
 * @param   None
 * @retval   :�Ƿ�ʱ
 * @author  peach99CPP
 ***********************************************************************/
bool Get_TimeResult(void)
{
  return OverTimeFlag;
}

/**********************************************************************
 * @Name    Deinit_Time
 * @declaration :���³�ʼ����ʱ����ر�����ɾ������ǰʹ��
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Deinit_Time(void)
{
  CountTimeTask_Exit = 1;   //��־�˳� ��ʱ��������п�������Ϊ��ʱ����Ҫ�ֶ���1
  now_time = wait_time = 0; //���������
}

/**********************************************************************
 * @Name    Start_CountTime
 * @declaration : ��ʼ��ʱ
 * @param   timelength: [����/��] ʱ�䳤��
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Start_CountTime(long timelength) //��ʼ����
{
  if (CountTimeTask_Exit && OverTimeHandle == NULL) //����һ���������н���ʱ
  {
    CountTimeTask_Exit = 0; //�����˳���־
    now_time = 0;           //���㵱ǰ��ʱ��
    OverTimeFlag = 0;       //�����ʱ��־λ
    wait_time = timelength;//����Ҫ�ȴ���ʱ�䣬��λms
    osThreadDef(CountTimeTask, CountTimeTaskFunc, osPriorityHigh, 0, 256); //��������ṹ��
    OverTimeHandle = osThreadCreate(osThread(CountTimeTask), NULL);        //��������
  }
  else 
  printf("��ʱ������ʧ��\r\n");
}

/**********************************************************************
 * @Name    CountTimeTaskFunc
 * @declaration :����ʵ�ֺ���
 * @param   argument: [����/��] ��
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void CountTimeTaskFunc(void const *argument)
{
  while (!CountTimeTask_Exit && !OverTimeFlag) //δ��ʱ�Լ�δ�˳�ʱ
  {
    while (now_time < wait_time && !CountTimeTask_Exit) //��ʱ����
    {
      now_time += 5; //�ۼ�ʱ��ֵ
      osDelay(5);    //����ϵͳ�ӳ� ���������л�
    }
    OverTimeFlag = 1; //��־��ʱ�Ѿ���ʱ
    osDelay(10);
  }
  Deinit_Time();         //���³�ʼ����ر���
  OverTimeHandle = NULL; //���н���  ������ÿ�
  vTaskDelete(NULL);     //ɾ������
}

/**********************************************************************
 * @Name    Exit_CountTime
 * @declaration : �˳���ʱ
 * @param   : [����/��] ��
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Exit_CountTime(void)
{
  if (OverTimeHandle != NULL)
  {
    CountTimeTask_Exit = 1; //��־�˳�
    wait_time = now_time = 0;
    OverTimeFlag = 1; //�����ʱ�ⲿ�����ڶ�ȡ
  }
}

int Get_CountTimeExit(void)
{
  return CountTimeTask_Exit;
}
