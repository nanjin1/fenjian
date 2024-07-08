#include "openmv.h"
#include "servo.h"
#include "chassis.h"
#include "QR_code.h"
#include "read_status.h"
#include "general.h"
#include "uart_handle.h"

#define STOP_SIGNAL 0XAABB
short Handle_Flag = 0;
int mv_param;
short AcIDofBar = 1;
int mv_one_flag = 1;
short mv_stop_flag = 0; //�ж�MV��صĹ���״̬
int Disc_Count = 0;
int rectangle_count = 0;
int servo_flag = 0;
osThreadId MV_QuertThreadHandle;
bool MV_QueryTask_Exit = true;
bool MV_Query_Stae = false;
uint8_t Rest_QueryTimes = 0;
int YuanPan_first_flag = 0;
int get_mv_param = 0;
int mv_one;
int cnt=0;
int stake = 0;
int stake_count = 0;
int yellow = 0;
int yuan_count = 0;



MvGo_t go_switch;
void cnt_clear(void)
{
   cnt=0;
}


int mv_go_flag(void)
{
   return mv_one;
}

bool Get_go_switch(void)
{
    return go_switch.motor_enable;
}

void Set_go_switch(bool go)
{
    go_switch.motor_enable = go;
}

void Set_FirstFlag(int state)
{
    YuanPan_first_flag = state;
}
int Get_FirstFlag(void)
{
    return YuanPan_first_flag;
}

/**
 * @description:  ����MV������
 * @return {*}
 */
void MV_QueryTask_Start(void)
{
    if (MV_QueryTask_Exit)
    {
        MV_QueryTask_Exit = false;
        MV_Query_Stae = true;
        osThreadDef(MV_QuertThreadHandle, MV_QueryTaskFunc, osPriorityHigh, 0, 256);
        MV_QuertThreadHandle = osThreadCreate(osThread(MV_QuertThreadHandle), NULL);
    }
}
void Exit_MV_QueryTask(void)
{
    MV_QueryTask_Exit = true;
}

void MV_QueryTaskFunc(void const *argument)
{
    while (!MV_QueryTask_Exit)
    {
        //���ÿ�ʼ��ʱ����
        if (MV_Query_Stae)
        {
            MV_SendCmd(9, 0); //��openmv���Ͳ�ѯָ��
            osDelay(100);     // 10HZ
        }
        else
        {
            osDelay(10); //��Ƶ��ˢ��
        }
    }
    vTaskDelete(NULL);
}
void Set_QueryState(int state)
{
    MV_Query_Stae = state;
    if (state == 1)
    {
        if (MV_QueryTask_Exit)
        {
            MV_QueryTask_Start();
        }
    }
}

void Update_rectangle_count(void)
{
    rectangle_count += 1;
}
int Get_RecCount(void)
{
    return rectangle_count;
}
mvrec_t mv_rec; // mv�Ľṹ��
mv_t MV =       //Ϊ�ṹ�帳��ֵ
    {
        .mv_uart = &huart4,
        .enable_switch = true,
        .mv_cmd = {0},
        .rec_buffer = {0},
        .rec_len = 0,
        .RX_Status = 0}; //��ʼ������

volatile int disc_countval = 0, color_val = 0;

void Set_AcIDofBar(short change)
{
    AcIDofBar = change;
}
void Disc_Report(void)
{
    disc_countval++;
    printf("\n\tԲ�̻�����,��ǰĿ��%d,������%d \t\n", color_val, disc_countval);
    if ((disc_countval >= 8 && color_val == 0) || (disc_countval >= 4 && color_val == 1))
    {
        disc_countval = 0;
        color_val++;
    }
}

void Set_MV_Mode(bool mode)
{
    MV.enable_switch = mode;
}
bool Get_MV_Mode(void)
{
    return MV.enable_switch;
}
/**********************************************************************
 * @Name    cmd_encode
 * @declaration : ����Э����뷢�͵�����
 * @param   event_id: [����/��]  �¼�������
 **			 param: [����/��]     ������16λ����
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void cmd_encode(const uint8_t event_id, uint8_t param)
{

    //����ͨѶЭ��
    MV.mv_cmd[0] = START_BYTE; //֡ͷ
    MV.mv_cmd[1] = event_id;   //�������¼�id
    MV.mv_cmd[2] = param;
    MV.mv_cmd[3] = (uint8_t)(event_id + param); //��У��
    MV.mv_cmd[4] = END_BYTE;                    //֡β
}
void MV_SendCmd(const uint8_t event_id, const int param)
{
    cmd_encode(event_id, param);                                 //���ݻ�õĲ�������cmd����
    HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff); //��cmd���ͳ�ȥ
printf("        ok\r\n");
    memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));                     //��cmd�������³�ʼ��
}

/**********************************************************************
 * @Name    MV_IRQ
 * @declaration :  openmvͨѶ���жϴ�����
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void MV_IRQ(void)
{
    uint8_t rec_data = MV.mv_uart->Instance->RDR;
    if (MV.RX_Status == 0)
    {
        if (rec_data == START_BYTE)
        {
            MV.RX_Status = 1; //����֡ͷ������ǣ�ֱ���˳�
            MV.rec_len = 0;
            return;
        }
        else if (rec_data == QR_StartByte)
        {
            MV.RX_Status = 2; //���յ�ת���Ķ�ά�������
            MV.rec_len = 0;   //��ʼ�����ձ���
            QR_LoadData(rec_data, 1);
        }
    }
    else if (MV.RX_Status == 1)
    {
        if (rec_data == END_BYTE && MV.rec_len == BUFFER_SIZE - 2)
        {
            MV_rec_decode();
            Enable_StopSignal();
            MV.RX_Status = 0;
        }
        else
        {
            MV.rec_buffer[MV.rec_len++] = rec_data; //��������
            if (MV.rec_len == MAX_REC_SIZE)
            {
                MV.RX_Status = 0; //��ֹ��Ϊ�����¿���
                MV.rec_len = 0;
                memset(MV.rec_buffer, 0, sizeof(MV.rec_buffer));
            }
        }
    }
    else if (MV.RX_Status == 2)
    {
        if (QR_LoadData(rec_data, 0))
        {
            MV.RX_Status = 0; //������� ����
            MV.rec_len = 0;
        }
    }
}

/**********************************************************************
 * @Name    MV_rec_decode
 * @declaration : �жϽ�����ɺ󣬶Խ��յ����ݽ��н���
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void MV_rec_decode(void)
{
    if (MV.rec_buffer[0] + MV.rec_buffer[1] == MV.rec_buffer[2])
    {
        mv_rec.event = MV.rec_buffer[0];
        mv_rec.param = MV.rec_buffer[1];
        MV_Decode();
        mv_rec.event = 0, mv_rec.param = 0; //ʹ��������
        MV.rec_len = 0;                     //����
        MV.RX_Status = 0;
    }
    //������֮��ǵ����³�ʼ���ṹ���е�rec_len��RX_status�������������
}

/****�����ǵײ�ʵ�֣��������ϲ��Ӧ��****/

/**********************************************************************
 * @Name    MV_Decode
 * @declaration :�����Լ�����Ĳ�������ִ������
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void MV_Decode(void)
{
#define pid_p 0.5
#define Ball_Signal 0x06
#define Rectangle_Signal 0x01
#define Ring_Signal 0x02
#define MV_Blob 0X02

    if (Get_Servo_Flag()) //���У����Խ���ָ�� ��ʱopenmv�Ͷ�ض�׼����ִ��ָ��
    {
        
        if ((mv_rec.event == MV_Blob)&&(Get_FirstFlag()==0))
        {
			if(stake == 0)
			{

				
  //              ActionGroup(83, 1);
		//		yuan_count = 0; 
				
				Set_FirstFlag(1);
     //           printf("����\n");
		//		yellow = 1;
				
				
			}
			else if(stake == 1)	
			{
				set_speed(0,0,0);
				stake_count = 1;
				
				
			}
        }

//��׮����mvʹ��

	
			
		
        else if (mv_rec.event == Ball_Signal && Get_MV_once() == 1) //�������
        {
            printf("�������ƽ̨����\n");
            printf("Ҫץ���\r\n");
            printf("ͣ��\r\n");
				Set_HeightAvailable(false);//���ø߶ȱ任
				get_mv_param = mv_rec.param;
			   Enable_StopSignal();//ʹ��ͣ���ź�
			   Set_QueryState(0);//�ر�mv���� 
				Set_MV_once(0);
			  if(cnt<5)
			  {
				  extern int color;
				  
				  	Enable_StopSignal(); //ʹ��ͣ���ź�
				  if(color == 0)
				  {
				  switch (Get_Height()) //��ȡ��ǰ�ĸ߶���Ϣ�����ݸ߶Ƚ���ץȡ
				 {
					case LowestHeight:
						 cnt++;
						 ActionGroup(Lowest, 1);
						 break;
					case MediumHeight:
						 cnt++;
						 ActionGroup(Medium, 1);
						 break;
					case HighestHeight:
						 cnt++;
						 ActionGroup(Highest, 1);
						 break;
					default:;
				 }
				}
				  else
				  {
				  
					   switch (Get_Height()) //��ȡ��ǰ�ĸ߶���Ϣ�����ݸ߶Ƚ���ץȡ
				 {
					case LowestHeight:
						 cnt++;
						 ActionGroup(13, 1);
						 break;
					case MediumHeight:
						 cnt++;
						 ActionGroup(23, 1);
						 break;
					case HighestHeight:
						 cnt++;
						 ActionGroup(33, 1);
						 break;
					default:;
				 }
				  
				  
				  
				  
				  }
			}
			  else
			  {
			     Disable_StopSignal();//��������ͼ����߿�
			  }
          
        }
    }
}

/**********************************************************************
 * @Name    Get_Stop_Signal
 * @declaration :���ش�ʱ�Ƿ�ֹͣ���ź�
 * @param   None
 * @retval   : �Ƿ�Ӧ��ͣ����ͣ����Ϊ1
 * @author  peach99CPP
 ***********************************************************************/
int Get_Stop_Signal(void)
{
    return mv_stop_flag;
}

/**********************************************************************
 * @Name    Enable_StopSignal
 * @declaration :ʹ��ͣ���ı�־λ
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Enable_StopSignal(void)
{
    mv_stop_flag = 1;
}

/**********************************************************************
 * @Name    Disable_StopSignal
 * @declaration : ���ͣ����־λ
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Disable_StopSignal(void)
{
    mv_stop_flag = 0;
}

/**********************************************************************
 * @Name    MV_Start
 * @declaration : Mv��ʼ��Ӧ����
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void MV_Start(void)
{
    Set_MV_Mode(true);
    MV_SendCmd(1, 0);
}

/**********************************************************************
 * @Name    MV_Stop
 * @declaration : ��MV����ֹͣ�ź�
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void MV_Stop(void)
{
    Set_MV_Mode(false);
    MV_SendCmd(0, 0);
}
void OpenMV_ChangeRoi(int roi)
{
    MV_SendCmd(11, roi);
    osDelay(100);
}

void Clear_MV_param(void)
{
    get_mv_param = 0;
}

int Get_MV_param(void)
{
    return get_mv_param;
}

void Set_MV_once(int flag)
{
    mv_one_flag = flag;
}

int Get_MV_once(void)
{
    return mv_one_flag;
}
