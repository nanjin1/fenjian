#include "servo.h "
#include "uart_handle.h"
#include "read_status.h"
uint8_t mv_rec_flag = 1; //初始状态，可以直接执行舵控指令
uint8_t LastID = 0X00;   //这个移动成全局变量
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

#define Action_Group_Run 0x06 //指定运行动作组的指令类型
#define Action_Group_Completed 0x08
#define ID_Max 100
#define FrameHead 0X55             //帧头
#define SpecialID 0xFF             //仅用于标记 后续根据实际需求更改
#define SpecialTime (uint16_t)0X00 //同上 组合使用
/**
 * @description:根据需要获取数据的高低8位
 * @param {uint16_t} obj 待转换对象
 * @param {char} type2trans转换类型
 * @return {uint8_t}提取出的8位数据
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
 * @description: 根据需要的动作组ID进行指令发送
 * @param {uint8_t} groupId动作组编号
 * @return {*} 无
 */
void ActionGroup(uint8_t groupId, uint16_t run_times)
{
    //绝大多数情况下 只运行一次动作组 因此此处默认为1次
    //如果需要多次运行的 后续加特殊判断即可
    uint8_t cmdLength = 0X05;    //数据长度的标志 固定的5
    uint16_t defaultTime = 0X01; //默认运行1次
    if (groupId > ID_Max)
    {
        Error_Report(2); //进行报错处理
        return;
    }
    else if (run_times == 0)
    {
        Error_Report(3); //进行报错处理
        return;
    }
    defaultTime *= run_times; //确认运行次数
    LastID = groupId;         //更新数据
    Disable_ServoFlag();      //拉低标志位
    //帧头的标记做好
    servo_controler.cmd_buffer[0] = servo_controler.cmd_buffer[1] = FrameHead;
    servo_controler.cmd_buffer[2] = cmdLength;        //数据产长度的标记
    servo_controler.cmd_buffer[3] = Action_Group_Run; //指令类型
    servo_controler.cmd_buffer[4] = groupId;          //动作组编号
    //取低8位
    servo_controler.cmd_buffer[5] = Get_Uint16_Transform(defaultTime, 'l');
    //取高8位
    servo_controler.cmd_buffer[6] = Get_Uint16_Transform(defaultTime, 'h');
    HAL_UART_Transmit(servo_controler.uart,
                      servo_controler.cmd_buffer,
                      cmdLength + 2,
                      0xff);
    memset(servo_controler.cmd_buffer, //清空 没啥大必要
           0,
           (cmdLength + 2) * sizeof(uint8_t));
}
/**
 * @description:舵控的中断处理函数
 * @return {*} 无
 */
void ServoInfBack_IRQ(void)
{
    //处理接收中断
    if (__HAL_UART_GET_IT(servo_controler.uart, UART_IT_RXNE))
    {
        static uint8_t rec_data;
        static uint8_t rec_state = 0;
        rec_data = servo_controler.uart->Instance->RDR;
        //帧头处理
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
        //正文处理
        if (rec_state == 2)
        {
            servo_controler.rec_buffer[servo_controler.rec_index++] = rec_data;
            if (servo_controler.rec_index >= servo_controler.rec_buffer[0])
            {

                if (servo_controler.rec_buffer[1] == Action_Group_Completed)
                {
                    //上一次动作组运行完成
                    if (servo_controler.rec_buffer[2] == LastID)
                    {
//                        printf("上一个动作组运行完成\n");
                        Enable_ServoFlag(); //拉高标志位
                    }
                    else //说明出错了
                    {
                        Error_Report(3); //报错
                    }
                }
                rec_state = 0;     //复位状态标志位
                Servo_Rx_Deinit(); //重新初始化接收铺的Index
            }
        }
    }
}
/**********************************************************************
 * @Name    Servo_Rx_Deinit
 * @declaration :重新初始化接收数组的下标
 * @param   None
 * @retval   :无
 * @author  peach99CPP
 ***********************************************************************/
void Servo_Rx_Deinit(void)
{
    servo_controler.rec_index = 0;
}
/**********************************************************************
 * @Name    Error_Report
 * @declaration : 进行错误报告
 * @param   type: [输入/出]  错误类型
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Error_Report(int type)
{
    if (type == 1)
        printf("角度值超限制\r\n");
    else if (type == 2)
        printf("动作组编号超限制\r\n");
    else if (type == 3)
    {
        printf("收到的动作组执行指令有误\n");
    }
    else if (type == 4)
        printf("运行次数不能为无限\r\n");
    else
        printf("舵控发生未知错误\r\n");
}

/**********************************************************************
 * @Name    Cmd_Convert
 * @declaration : 将传入的参数编码成单个数字格式
 * @param   cmd: [输入/出]
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Cmd_Convert(int cmd)
{
    short temp_num[5] = {0}, index = 0;
    if (cmd == 0) //对为0的参数进行单独的处理
    {
        servo_controler.cmd_buffer[servo_controler.current_index++] = 0;
        return;
    }
    while (cmd != 0) //获取其数字的每一位，转化成字符
    {
        temp_num[index++] = cmd % 10 + '0';
        cmd /= 10;
    }
    index -= 1; //取得对应的下标
    while (index >= 0)
        servo_controler.cmd_buffer[servo_controler.current_index++] = temp_num[index--];
    return;
}

/**********************************************************************
 * @Name    Servo_Uart_Send
 * @declaration :串口数据发送函数
 * @param   None
 * @retval   :无
 * @author  peach99CPP
 ***********************************************************************/
void Servo_Uart_Send(void)
{
    if (servo_controler.current_index == 0)
        return;
    //打包数据，加上结尾
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0D;
    servo_controler.cmd_buffer[servo_controler.current_index++] = 0X0A;
    //发送
    HAL_UART_Transmit(servo_controler.uart, servo_controler.cmd_buffer, servo_controler.current_index, 0xff);
    //重新初始化结构体变量
    memset(servo_controler.cmd_buffer, 0, servo_controler.current_index * sizeof(uint8_t));
    servo_controler.current_index = 0;
}

/**********************************************************************
 * @Name    Single_Control
 * @declaration :  控制单个舵机转动的函数
 * @param   id: [输入/出]  电机编号
 **			 control_mode: [输入/出]  控制模式，目前共三种，看下方的注释
 **			 angle: [输入/出]       目标角度（根据控制模式不同，有不同的输入范围
 **			 time: [输入/出]   用多少时间来执行
 **			 delay: [输入/出]  执行结束后延迟多久
 * @retval   :   无
 * @author  peach99CPP
 ***********************************************************************/
void Single_Control(int id, int control_mode, int angle, int time, int delay)
{
    if (id > 24 || control_mode > 3 || servo_controler.current_index != 0)
        return;
    //设置电机编号
    servo_controler.cmd_buffer[servo_controler.current_index++] = '#';
    Cmd_Convert(id);

    /*根据控制模式设置要转到的角度
    mode 1: 直接原生角度，此时角度值应该在500-2500范围
    mode 2: 将输入的角度值转换到180舵机对应的角度值
    mode 3: 将输入的角度值转换到270舵机对应的角度值
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
    else if (control_mode == 2) //设置的是180度对应的角度
    {
        if (angle > 180 || angle < 0)
        {
            Error_Report(1);
            return;
        }
        angle = 500 + (2000.0 / 180.0) * angle; //转换到180度范围下对应的角度数值
    }
    else if (control_mode == 3)
    {
        if (angle > 270 || angle < 0)
        {
            Error_Report(1);
            return;
        }
        angle = 500 + (2000.0 / 270.0) * angle; //满角度为270度的舵机的指令
    }
    Disable_ServoFlag(); //把标志位设置好
    Cmd_Convert(angle);

    //设置执行时间
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'T';
    Cmd_Convert(time);
    //设置指令延迟
    servo_controler.cmd_buffer[servo_controler.current_index++] = 'D';
    Cmd_Convert(delay);
    //通过串口发送
    Servo_Uart_Send();
}


/**********************************************************************
 * @Name    Servo_RX_IRQ
 * @declaration : 舵控的中断处理函数
 * @param   None
 * @retval   : 无
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
 * @declaration :获取舵控运行状态
 * @param   None
 * @retval   : 舵机运行结束为1 否则为0
 * @author  peach99CPP
 ***********************************************************************/
int Get_Servo_Flag(void)
{
    return mv_rec_flag;
}

/**********************************************************************
 * @Name    Enable_ServoFlag
 * @declaration :使能舵机标志位，表明此时舵机运行结束
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Enable_ServoFlag(void)
{
    mv_rec_flag = 1;
}

/**********************************************************************
 * @Name    Disable_ServoFlag
 * @declaration : 清除舵机标志位，说明此时多斤正在运行中
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Disable_ServoFlag(void)
{
    mv_rec_flag = 0;
}

/****以下是具体实现函数****/
#include "Wait_BackInf.h"
#include "cmsis_os.h"
#include "chassis.h"
#define Wait_Time 3000

/**********************************************************************
 * @Name    Wait_Servo_Signal
 * @declaration :等待函数信号
 * @param   wait_time_num: [输入/出] 超时值
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Wait_Servo_Signal(long wait_time_num)
{
    long temp_time = 100;
    osDelay(temp_time); //最小停留100ms 再看看后面有无停留需要
    set_speed(0, 0, 0); //等待过程中是停车的
    while (Get_Servo_Flag() == false && temp_time < wait_time_num)
    {
        temp_time += 5;
        osDelay(5);
    }
}
