#include "servo.h "
#include "uart_handle.h"
#include "read_status.h"
uint8_t mv_rec_flag = 1; //��ʼ״̬������ֱ��ִ�ж��ָ��
uint8_t LastID = 0X00;   //����ƶ���ȫ�ֱ���
ServoControler_t servo_controler =
    {
        .uart = &huart5,
        .current_index = 0,
        .cmd_buffer = {0},
        .rec_buffer = {0},
        .rec_index = 0};

bool if_up = false;
void Set_IFUP(bool status)
{
    if_up = status;
}
bool Get_IFUP(void)
{
    return if_up;
}

#define Action_Group_Run 0x06 //ָ�����ж������ָ������
#define Action_Group_Completed 0x08
#define ID_Max 100
#define FrameHead 0X55             //֡ͷ
#define SpecialID 0xFF             //�����ڱ�� ��������ʵ���������
#define SpecialTime (uint16_t)0X00 //ͬ�� ���ʹ��
/**
 * @description:������Ҫ��ȡ���ݵĸߵ�8λ
 * @param {uint16_t} obj ��ת������
 * @param {char} type2transת������
 * @return {uint8_t}��ȡ����8λ����
 */

uint8_t Get_Uint16_Transform(uint16_t obj, char type2trans)
{
    if (type2trans == 'h' || type2trans == 'H')
    {
        return (uint8_t)(obj >> 8);
    }
    else if (type2trans == 'l' || type2trans == 'L')
    {
        return (uint8_t)(obj);
    }
    return 0;
}
/**
 * @description: ������Ҫ�Ķ�����ID����ָ���
 * @param {uint8_t} groupId��������
 * @return {*} ��
 */
void ActionGroup(uint8_t groupId, uint16_t run_times)
{
    //������������ ֻ����һ�ζ����� ��˴˴�Ĭ��Ϊ1��
    //�����Ҫ������е� �����������жϼ���
    uint8_t cmdLength = 0X05;    //���ݳ��ȵı�־ �̶���5
    uint16_t defaultTime = 0X01; //Ĭ������1��
    if (groupId > ID_Max)
    {
        Error_Report(2); //���б�����
        return;
    }
    else if (run_times == 0)
    {
        Error_Report(3); //���б�����
        return;
    }
    defaultTime *= run_times; //ȷ�����д���
    LastID = groupId;         //��������
    Disable_ServoFlag();      //���ͱ�־λ
    //֡ͷ�ı������
    servo_controler.cmd_buffer[0] = servo_controler.cmd_buffer[1] = FrameHead;
    servo_controler.cmd_buffer[2] = cmdLength;        //���ݲ����ȵı��
    servo_controler.cmd_buffer[3] = Action_Group_Run; //ָ������
    servo_controler.cmd_buffer[4] = groupId;          //��������
    //ȡ��8λ
    servo_controler.cmd_buffer[5] = Get_Uint16_Transform(defaultTime, 'l');
    //ȡ��8λ
    servo_controler.cmd_buffer[6] = Get_Uint16_Transform(defaultTime, 'h');
    HAL_UART_Transmit(servo_controler.uart,
                      servo_controler.cmd_buffer,
                      cmdLength + 2,
                      0xff);
    memset(servo_controler.cmd_buffer, //��� ûɶ���Ҫ
           0,
           (cmdLength + 2) * sizeof(uint8_t));
}
/**
 * @description:��ص��жϴ�����
 * @return {*} ��
 */
void ServoInfBack_IRQ(void)
{
    //��������ж�
    if (__HAL_UART_GET_IT(servo_controler.uart, UART_IT_RXNE))
    {
        static uint8_t rec_data;
        static uint8_t rec_state = 0;
        rec_data = servo_controler.uart->Instance->RDR;
        //֡ͷ����
        if (rec_state == 0 && rec_data == FrameHead)
        {
            rec_state = 1;
            return;
        }
        else if (rec_state == 1 && rec_data == FrameHead)
        {
            servo_controler.rec_index = 0;
            rec_state = 2;
            return;
        }
        //���Ĵ���
        if (rec_state == 2)
        {
            servo_controler.rec_buffer[servo_controler.rec_index++] = rec_data;
            if (servo_controler.rec_index >= servo_controler.rec_buffer[0])
            {

                if (servo_controler.rec_buffer[1] == Action_Group_Completed)
                {
                    //��һ�ζ������������
                    if (servo_controler.rec_buffer[2] == LastID)
                    {
//                        printf("��һ���������������\n");
                        Enable_ServoFlag(); //���߱�־λ
                    }
                    else //˵��������
                    {
                        Error_Report(3); //����
                    }
                }
                rec_state = 0;     //��λ״̬��־λ
                Servo_Rx_Deinit(); //���³�ʼ�������̵�Index
            }
        }
    }
}
/**********************************************************************
 * @Name    Servo_Rx_Deinit
 * @declaration :���³�ʼ������������±�
 * @param   None
 * @retval   :��
 * @author  peach99CPP
 ***********************************************************************/
void Servo_Rx_Deinit(void)
{
    servo_controler.rec_index = 0;
}
/**********************************************************************
 * @Name    Error_Report
 * @declaration : ���д��󱨸�
 * @param   type: [����/��]  ��������
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Error_Report(int type)
{
    if (type == 1)
        printf("�Ƕ�ֵ������\r\n");
    else if (type == 2)
        printf("�������ų�����\r\n");
    else if (type == 3)
    {
        printf("�յ��Ķ�����ִ��ָ������\n");
    }
    else if (type == 4)
        printf("���д�������Ϊ����\r\n");
    else
        printf("��ط���δ֪����\r\n");
}

/**********************************************************************
 * @Name    Cmd_Convert
 * @declaration : ������Ĳ�������ɵ������ָ�ʽ
 * @param   cmd: [����/��]
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Cmd_Convert(int cmd)
{
    short temp_num[5] = {0}, index = 0;
    if (cmd == 0) //��Ϊ0�Ĳ������е����Ĵ���
    {
        servo_controler.cmd_buffer[servo_controler.current_index++] = 0;
        return;
    }
    while (cmd != 0) //��ȡ�����ֵ�ÿһλ��ת�����ַ�
    {
        temp_num[index++] = cmd % 10 + '0';
        cmd /= 10;
    }
    index -= 1; //ȡ�ö�Ӧ���±�
    while (index >= 0)
        servo_controler.cmd_buffer[servo_controler.current_index++] = temp_num[index--];
    return;
}

/**********************************************************************
 * @Name    Servo_Uart_Send
 * @declaration :�������ݷ��ͺ���
 * @param   None
 * @retval   :��
 * @author  peach99CPP
 ***********************************************************************/
void Servo_Uart_Send(void)
{
    if (servo_controler.current_index == 0)
        return;
    //������ݣ����Ͻ�β
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0D;
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0A;
    //����
    HAL_UART_Transmit(servo_controler.uart, servo_controler.cmd_buffer, servo_controler.current_index, 0xff);
    //���³�ʼ���ṹ�����
    memset(servo_controler.cmd_buffer, 0, servo_controler.current_index * sizeof(uint8_t));
    servo_controler.current_index = 0;
}

/**********************************************************************
 * @Name    Single_Control
 * @declaration :  ���Ƶ������ת���ĺ���
 * @param   id: [����/��]  ������
 **			 control_mode: [����/��]  ����ģʽ��Ŀǰ�����֣����·���ע��
 **			 angle: [����/��]       Ŀ��Ƕȣ����ݿ���ģʽ��ͬ���в�ͬ�����뷶Χ
 **			 time: [����/��]   �ö���ʱ����ִ��
 **			 delay: [����/��]  ִ�н������ӳٶ��
 * @retval   :   ��
 * @author  peach99CPP
 ***********************************************************************/
void Single_Control(int id, int control_mode, int angle, int time, int delay)
{
    if (id > 24 || control_mode > 3 || servo_controler.current_index != 0)
        return;
    //���õ�����
    servo_controler.cmd_buffer[servo_controler.current_index++] = '#';
    Cmd_Convert(id);

    /*���ݿ���ģʽ����Ҫת���ĽǶ�
    mode 1: ֱ��ԭ���Ƕȣ���ʱ�Ƕ�ֵӦ����500-2500��Χ
    mode 2: ������ĽǶ�ֵת����180�����Ӧ�ĽǶ�ֵ
    mode 3: ������ĽǶ�ֵת����270�����Ӧ�ĽǶ�ֵ
    */
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'P';

    if (control_mode == 1)
    {
        if (angle > 2500 || angle < 500)
        {
            Error_Report(1);
            return;
        }
    }
    else if (control_mode == 2) //���õ���180�ȶ�Ӧ�ĽǶ�
    {
        if (angle > 180 || angle < 0)
        {
            Error_Report(1);
            return;
        }
        angle = 500 + (2000.0 / 180.0) * angle; //ת����180�ȷ�Χ�¶�Ӧ�ĽǶ���ֵ
    }
    else if (control_mode == 3)
    {
        if (angle > 270 || angle < 0)
        {
            Error_Report(1);
            return;
        }
        angle = 500 + (2000.0 / 270.0) * angle; //���Ƕ�Ϊ270�ȵĶ����ָ��
    }
    Disable_ServoFlag(); //�ѱ�־λ���ú�
    Cmd_Convert(angle);

    //����ִ��ʱ��
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'T';
    Cmd_Convert(time);
    //����ָ���ӳ�
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'D';
    Cmd_Convert(delay);
    //ͨ�����ڷ���
    Servo_Uart_Send();
}


/**********************************************************************
 * @Name    Servo_RX_IRQ
 * @declaration : ��ص��жϴ�����
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Servo_RX_IRQ(void)
{
    if (__HAL_UART_GET_IT(servo_controler.uart, UART_IT_RXNE))
    {
        static uint8_t rec_data;
        rec_data = servo_controler.uart->Instance->RDR;
        servo_controler.rec_buffer[servo_controler.rec_index++] = rec_data;
        if (servo_controler.rec_index >= 2)
        {
            if (servo_controler.rec_buffer[0] == 'O' && servo_controler.rec_buffer[1] == 'K')
            {
                Enable_ServoFlag();
                Servo_Rx_Deinit();
            }
        }
    }
}

/**********************************************************************
 * @Name    Get_Servo_Flag
 * @declaration :��ȡ�������״̬
 * @param   None
 * @retval   : ������н���Ϊ1 ����Ϊ0
 * @author  peach99CPP
 ***********************************************************************/
int Get_Servo_Flag(void)
{
    return mv_rec_flag;
}

/**********************************************************************
 * @Name    Enable_ServoFlag
 * @declaration :ʹ�ܶ����־λ��������ʱ������н���
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Enable_ServoFlag(void)
{
    mv_rec_flag = 1;
}

/**********************************************************************
 * @Name    Disable_ServoFlag
 * @declaration : ��������־λ��˵����ʱ�������������
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Disable_ServoFlag(void)
{
    mv_rec_flag = 0;
}

/****�����Ǿ���ʵ�ֺ���****/
#include "Wait_BackInf.h"
#include "cmsis_os.h"
#include "chassis.h"
#define Wait_Time 3000

/**********************************************************************
 * @Name    Wait_Servo_Signal
 * @declaration :�ȴ������ź�
 * @param   wait_time_num: [����/��] ��ʱֵ
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Wait_Servo_Signal(long wait_time_num)
{
    long temp_time = 100;
    osDelay(temp_time); //��Сͣ��100ms �ٿ�����������ͣ����Ҫ
    set_speed(0, 0, 0); //�ȴ���������ͣ����
    while (Get_Servo_Flag() == false && temp_time < wait_time_num)
    {
        temp_time += 5;
        osDelay(5);
    }
}
