
#include "general_interface.h"
#include "read_status.h "
#include "openmv.h"
#define Wait_Dealy_MAX 15000
#define Line_Type 1
#define Encoder_Type 2
#define YuanPanDelay  10000
#define Nomal 0
#define Rightside 1
#define Leftside 2
#define Low 190
#define High 200

int color = 0;



int flag_pillars = 0;


/**
 * @description: ͨ�����ú����������������
 * @param {int} idʹ���ĸ�����
 * @param {int} num �����ٸ�����
 * @param {int} speed��ʲô�ٶ�����
 * @return {*}                              �����ã���û��Ҫ
 */
void Move_CountBar(int id, int num, int speed)
{
    if (id != 0 && id != 2 && id != 9)
    {
    //    printf("ѡ��ĺ���id����");while ()
    {
    }
        return;
    }
    if (id != 9)
    {
        Init_BarCount(id);
        set_imu_status(1);      // TODO ȷ�������ǿ���״̬
        set_speed(0, speed, 0); // TODO ������ʽ�����ٶ�
        while (Get_BarCount(id) < num)
        {
            osDelay(5);
        }
        osDelay(50);
    }
    else
    {
        // id = 2;
        Init_BarCount(id);
        set_imu_status(1);      // TODO ȷ�������ǿ���״̬
        set_speed(0, speed, 0); // TODO ������ʽ�����ٶ�
        while (Get_BarCount(id) < num)
        {
            osDelay(5);
        }
        osDelay(200);
    }
    set_speed(0, 0, 0);
    osDelay(100);
}




//��ʱ��·������б��ʲô�ĺ�����������
void Move_inch(int dir1,int dir2,int time)
{
	float temp_time = 50;
	
	set_speed(dir1,dir2,0);
	
	osDelay(50);
	while(temp_time < time)               //osDelay�����Ĳ�����֧�ֳ�ʱ�����ʱ�����Ծͼ�ʵ��һ����׼ȷ����ʱ
	{
	temp_time+=5;
	osDelay(5);	
	}

	set_speed(0,0,0);
}



//������ߵ�ѭ��ģ���ȡ
int Get_Home1(void)
{
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==RESET)
		return 1;
    else
		return 0;

}
/****************************************
 *
 *
 *	ѭ��ģ��ؼ�
 *
 ****************************************/
//����ǰ���ѭ��ģ���ȡ
int Get_Home2(void)
{
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)==RESET)
		return 1;
    else
		return 0;

}

//��������ǵײ���߶�λ���ĺ���
int Get_HW_NEW1(void)
{
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)==RESET)
		return 1;//����������
    else
		return 0;//����������

}

/****************************************
 *	����ע�����
 *������׮�Ĺؼ����ڵ��������������λ��
 *����ǵ�����԰���ٶ�/�Ƕȵȵ�
 *
 ****************************************/

//��׮��ߺ���
int Get_HW_LEFT(void)
{
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)==RESET)
		return 1;//����������
    else
		return 0;//����������

}
//��׮�ұߺ���
int Get_HW_RIGHT(void)
{
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)==RESET)
		return 1;//����������
    else
		return 0;//����������

}

/****************************************
 *	�ߵ�Բ�̻���λ���ĵ�һ���׶���
 *	���Ű汾�ĸ��£��������ջ���ѡ������򵥴ֱ���һ�ְ취
 *	����ע������
 *	��ص������Լ��������ʱʱ��
 ****************************************/

void centren_control(int color)
{
	if(color == 0)
	{
	ActionGroup(2, 1); //�ı�洢����
	set_imu_param(900,100,0);
	Move_inch(450,300,1200);
	
	ActionGroup(81,1);

	Move_inch(0,400,2700);
	
	
	set_imu_param(80,33,0);
	
	while(Get_HW(2)==0)
	{
		set_speed(0,120,0);
	
	}
	
	set_speed(0,0,0);
	
	osDelay(70);
	
	set_imu_status(0);
	set_speed(120,0,0);
	osDelay(400);


	}
	
	//�쳡�Ĵ���
	else
	{
	ActionGroup(2, 1); //�ı�洢����
	set_imu_param(900,100,0);
	Move_inch(450,-400,1150);
	
	ActionGroup(81,1);
	
	Move_inch(0,-400,2700);

	set_imu_param(80,33,0);
		
	while(Get_HW(0)==0)
	{
		set_speed(0,-130,0);
	
	}
	
	set_speed(0,0,0);
	
	osDelay(90);
	
	
	set_imu_status(0);
	set_speed(120,0,0);
	//����װ����
	osDelay(500);
	
	
	
	
	}
	
//	Set_InitYaw(0);
}

/****************************************
 *	ץ���һϵ�ж���
 *	0��1��2 �����
 *	ϸ�ھ���Ҫ��Ҫ�Լ��ѿ���
 *
 ****************************************/

void control_ball(int ball_color)
{
	
	extern int yellow ;
	yellow = 1;

    set_speed(140,0,0);
	osDelay(400);
	set_speed(0,0,0);
		
	
	
	for(uint8_t i=0;i<2;i++)
		{
		 MV_SendCmd(1, 0); //ʹ��mv
		 osDelay(20);
		 MV_SendCmd(2, 1); //���û�ɫ��ΪĿ����
		 osDelay(20);
		}
		
	

	Set_QueryState(1);
		int count = 0;
		int i = 0;
		extern int yuan_count ;
		
		
		if(ball_color == 0){
    ActionGroup(82, 1); //���۵�״̬�����е��,��Ҫ����
    Wait_Servo_Signal(Wait_Dealy_MAX);
		}
		else
		{
		
		  ActionGroup(84, 1); //���۵�״̬�����е��,��Ҫ����
    Wait_Servo_Signal(Wait_Dealy_MAX);
		
		
		}
//	ActionGroup(2, 1); //�ı�洢����
//    Wait_Servo_Signal(Wait_Dealy_MAX);
	
		while(1)
	{
		
	   if(Get_FirstFlag()==1)
	   {
		  if(ball_color == 0)
		  {
			ActionGroup(83, 1);
			Wait_Servo_Signal(Wait_Dealy_MAX);	
		  }
		  else
		  {
		  ActionGroup(85, 1);
			Wait_Servo_Signal(Wait_Dealy_MAX);	
		  
		  }
			osDelay(200);
		   
		    Set_FirstFlag(0);
		
		  
	   }
	 
		 count++;
	   osDelay(10);
	   if(count>=200)
	      break;
   }
	
    Set_QueryState(0);
  
	osDelay(100);
	Enable_ServoFlag(); //���߱�־λ(1);
    yellow = 0;
	 
	 ActionGroup(3, 1); //�ı�洢����
    Wait_Servo_Signal(Wait_Dealy_MAX);
	for(uint8_t i=0;i<3;i++)
	{
	 MV_SendCmd(1, 0); //ʹ��mv
	 osDelay(20);
	 MV_SendCmd(2, ball_color); //������ɫΪĿ����
	 osDelay(20);
	}
	 i=0;
	 Set_QueryState(1);
	  Set_FirstFlag(0);
	 osDelay(280);
    
	while(1)
	{
		
	   if(Get_FirstFlag()==1)
	   {
		   printf("ɨ���¼�\n");
 if(ball_color == 0)
		  {
			ActionGroup(83, 1);
			Wait_Servo_Signal(Wait_Dealy_MAX);	
		  }
		  else
		  {
		  ActionGroup(85, 1);
			Wait_Servo_Signal(Wait_Dealy_MAX);	
		  
		  }
			osDelay(200);
		   
		    Set_FirstFlag(0);
	   }
	   
	   i++;
	   osDelay(10);
	   if(i>=200)
	   {
	      break;
	   }
	}
  

   // osDelay(YuanPanDelay);
    Set_QueryState(0);
	 osDelay(280);
	 Enable_ServoFlag(); //���߱�־λ(1);
    MV_SendCmd(0, 0);
   for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_imu_status(1);
  //  printf("�������β���\n");
    set_speed(0, 0, 0);
	
    ActionGroup(81, 1); //���۵�״̬�����е��
    Wait_Servo_Signal(Wait_Dealy_MAX);
	set_imu_status(1);
    osDelay(500);



}
/****************************************
 *	����λ
 *
 *	����ֱ��д��
 *	
 ****************************************/
void Dump_ball(int color)
{
	if(color == 0)
	{
		
	ActionGroup(0, 1);//�����е��
		
	move_by_encoder(1,-232);	
	Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
	
	
    Wait_Servo_Signal(Wait_Dealy_MAX);
	
	Turn_angle(1,180,0);
		
	Move_inch(0,100,350);	
		
		osDelay(400);
//	pillars_change(0);
//  pillars_close();
//  osDelay(500);
	
//  printf("���ϳɹ�\r\n");
	
	Move_inch(-100,0,1300);
	Move_inch(0,150,2300);
	
	
	set_imu_status(0);
	set_speed(-90,0,0);//װ����������̬
	osDelay(1500);
	set_speed(0,0,0);
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_imu_status(1);
	
	ActionGroup(35,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	set_speed(0,43,0);
	while(1)
	{
		
        if(Get_HW(1)==0)
		{
			set_speed(0, 0, 0);
	//		printf("���ҵ��������B����\r\n");
			break;
		}
	}
	ActionGroup(4,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	osDelay(2000);
	ActionGroup(5,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	ActionGroup(36,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	set_imu_status(0);
	set_speed(-90,0,0);//װ����������̬
	osDelay(1500);
	set_speed(0,0,0);
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_imu_status(1);
	
	
	}
	
	
	
	else
	{
	ActionGroup(0, 1);//�����е��
	
	move_by_encoder(1,-232);	
	Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
	
    Wait_Servo_Signal(Wait_Dealy_MAX);
	Turn_angle(1,180,0);
		
	Move_inch(0,-100,350);	
	
		osDelay(400);
		
//	pillars_change(0);
//  pillars_close();
//  osDelay(500);
	
//  printf("���ϳɹ�\r\n");
	
	Move_inch(-100,0,1300);
	Move_inch(0,-150,2050);
	
	set_imu_status(0);
	set_speed(-90,0,0);//װ����������̬
	osDelay(1500);
	set_speed(0,0,0);
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_imu_status(1);
	
	ActionGroup(35,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	set_speed(0,43,0);
	while(1)
	{
		
        if(Get_HW(1)==0)
		{
			set_speed(0, 0, 0);
			break;
		}
		}
	ActionGroup(4,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	osDelay(2000);
	ActionGroup(5,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	ActionGroup(36,1);
		
	set_imu_status(0);
	set_speed(-90,0,0);//װ����������̬
	osDelay(1500);
	set_speed(0,0,0);
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_imu_status(1);
		
	}
}
//����ľ�鶨λ

void Dump_block(void)
{
	
	
	set_imu_status(0);
	set_speed(-90,0,0);//װ����������̬
	osDelay(500);
	set_speed(0,0,0);
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_imu_status(1);
	
	
	ActionGroup(35,1);//��ľ�鶯�������	
	Wait_Servo_Signal(Wait_Dealy_MAX);
	set_speed(-10,38,0);
	osDelay(500);
	while(1)
	{
		
        if(Get_HW(1)==0)
		{
			set_speed(0, 0, 0);
		
			break;
		}
	}
	ActionGroup(4,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	osDelay(2000);
	ActionGroup(5,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	ActionGroup(34,1);//��ľ�鶯�������
	Wait_Servo_Signal(Wait_Dealy_MAX);
	 set_speed(-12,-66,0);
	while(1)
	{
		
        if(Get_HW(1)==1)
		{
			set_speed(0, 0, 0);
	//		printf("���ҵ����ڻ�ľ��B����\r\n");
			break;
		}
	}
	ActionGroup(6,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	osDelay(1000);
	ActionGroup(5,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	ActionGroup(36,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	
}

	
	
//����ⶨλ
void Dump_RoundRing(int color)
{
	if(color == 0)//��ɫ
	{
	set_imu_status(1);
	
	ActionGroup(1,1);
	Move_inch(0,130,450);
//�������ƽ̨�ֿ��ǱߵĻ�е��
	
	
	
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	set_imu_status(1);
	Move_inch(130,0,2800);
	
	set_speed(0,0,0);
	osDelay(1000);
	Turn_angle(1,180,0);//ת����ȷ�ĽǶ�
	//osDelay(1000);
	set_speed(0,150,0);
	osDelay(800);
	set_speed(0,0,0);
//	while(1)
//	{
//		
//        if(Get_HW_NEW1()==1)
//		{
//			set_speed(0, 0, 0);
//			
//			break;
//		}
//	}
	set_imu_status(0);
	ActionGroup(52,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	set_speed(-90,0,0);//װ����������̬
	osDelay(1000);	
	set_speed(0,0,0);
	set_speed(-70,0,0);//װ����������̬
	osDelay(1000);	
	set_speed(0,0,0);
	
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_imu_status(1);
//	
//	set_speed(-12,-30,0);
//	while(1)
//	{
//		
//        if(Get_HW_NEW1()==0)
//		{
//			set_speed(0, 0, 0);
//		//	printf("���ҵ���������������\r\n");
//			break;
//		}
//	}
//	osDelay(1000);
//	set_imu_status(0);
//	set_speed(-80,0,0);//װ����������̬
//	osDelay(1000);	
//	set_speed(0,0,0);
//	osDelay(500);
//	
//	 for(uint8_t i=0;i<3;i++)
//		{
//		 
//		 Set_InitYaw(0);
//		 osDelay(50);
//		
//		}
//	set_imu_status(1);
//	ActionGroup(63,1);
//	Wait_Servo_Signal(30000);
//	ActionGroup(52,1);
//	Wait_Servo_Signal(Wait_Dealy_MAX);
	}
	else
	{
	set_imu_status(1);
	ActionGroup(1,1);
	Move_inch(0,130,450);
	
	
	Move_inch(130,0,2800);
	
	set_speed(0,0,0);
	osDelay(1000);
	Turn_angle(1,180,0);//ת����ȷ�ĽǶ�
	//osDelay(1000);
	set_speed(0,150,0);
		osDelay(800);
		set_speed(0,0,0);
//	while(1)
//	{
//		
//        if(Get_HW_NEW1()==1)
//		{
//			set_speed(0, 0, 0);
//			
//			break;
//		}
//	}
//	
	set_imu_status(0);
	ActionGroup(52,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	set_speed(-90,0,0);//װ����������̬
	osDelay(1000);	
	set_speed(0,0,0);
	set_speed(-70,0,0);//װ����������̬
	osDelay(1000);	
	set_speed(0,0,0);
	osDelay(500);
	
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_imu_status(1);

//	set_speed(-12,-35,0);
//	while(1)
//	{
//		
//        if(Get_HW_NEW1()==0)
//		{
//			set_speed(0, 0, 0);
//			break;
//		}
//	}
//	set_imu_status(0);
//	set_speed(-80,0,0);//װ����������̬
//	osDelay(1000);	
//	set_speed(0,0,0);
//	osDelay(500);
//	
//	 for(uint8_t i=0;i<3;i++)
//		{
//		 
//		 Set_InitYaw(0);
//		 osDelay(50);
//		
//		}
//	set_imu_status(1);
//	ActionGroup(62,1);
//	Wait_Servo_Signal(30000);
//	ActionGroup(52,1);
//	Wait_Servo_Signal(Wait_Dealy_MAX);
	}
}

//������һ�ν���ƽ̨���ץȡ��ľ��
	
void Restart_Ladder()
{
	
	ActionGroup(52,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	Move_inch(0,-130,800);
	set_speed(-75,0,0);
	osDelay(1000);
	
	set_speed(0,-60,0);
	
	while(1)
	{
		if(Get_HW_others(3)==0)
		{
			set_speed(0,0,0);
		//	printf("2222");
			break;
		}
	}
	
	set_speed(0,40,0);
	
	while(1)
	{
		if(Get_HW_others(3)==1)
		{
			set_speed(0,0,0);
		//	printf("2222");
			break;
		}
	}
	set_speed(-75,0,0);
	osDelay(1000);
	
	
	
	ActionGroup(53,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	


}


void run_ladder(int color)
{
	//�ӵ���Ĳֿ��ߵ�����ƽ̨
	if(color == 0)
	{
		

		
		
		ActionGroup(52,1);//��ߴ�ɨ���ֹ������̬��ʱ��ײ������ƽ̨
		
		
		set_speed(-100,0,0);//ײ����������̬
		osDelay(2000);
		set_speed(0,0,0);
		 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
		
		Wait_Servo_Signal(Wait_Dealy_MAX);
		
		set_speed(-9,-48,0);
		osDelay(500);
		while(1)
		{
			if(Get_HW_others(3)==0)
			{
				set_speed(0,0,0);
		//		printf("2222");
				break;
			}
		}
		
		
	
		ActionGroup(53,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		
		set_motor_pid(90,200,9);
		
		
		Ladder_process(1,1,0);//��ʼ��һ�ν���ƽ̨
		Restart_Ladder();
					
		set_motor_pid(90,100,9);
		Ladder_process(1,1,1);
	//	printf("3333");

	}
	
	
	else
	{
	

		ActionGroup(52,1);//��ߴ�ɨ���ֹ������̬��ʱ��ײ������ƽ̨
		
		
		set_speed(-100,0,0);//ײ����������̬
		osDelay(2000);
		set_speed(0,0,0);
		 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
		Wait_Servo_Signal(Wait_Dealy_MAX);
		
		set_speed(-9,-48,0);
		osDelay(500);
		while(1)
		{
			if(Get_HW_others(3)==0)
			{
				set_speed(0,0,0);
		//		printf("2222");
				break;
			}
		}
		
		
		ActionGroup(53,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		
		set_motor_pid(90,200,9);
		
		Ladder_process(0,1,0);//��ʼ��һ�ν���ƽ̨
		
		Restart_Ladder();
					
		Ladder_process(0,1,1);

	}
	
	
	
	
	
}



void run_home(int color)
{
	
	extern int GO_home;
	
	if(color == 0)
	{
		GO_home = 1;
	set_speed(-90,0,0);//ײ����������̬
	osDelay(1200);
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}

	ActionGroup(10,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	move_by_encoder(2,630);
	Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
	Turn_angle(2,-90,0);
	osDelay(500);
	move_by_encoder(2,210);
	Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
		
	Turn_angle(2,-90,0);
		
	int i = 0;
	while(1)
	{
		Start_home:
		set_speed(-90,0,0);
		
		if(Get_Home1()==0)
		{
			set_speed(0,0,0);
			Turn_angle(2,-90,0);
			osDelay(100);
			if(Get_Home1()==1)goto Start_home;
			
			while(1)
			{

			Move_inch(90,0,150);
			Turn_angle(2,-90,0);
				
			if(Get_Home1()==1)
				{
					Move_inch(90,0,220);
					Turn_angle(2,-90,0);
					break;
				}
					
	
			}
					
			
			
			set_speed(0,0,0);
			break;
			
		}
	}
	
	Turn_angle(2,-90,0);		
		set_speed(0,30,0);
	
	while(1)
	{
		if(Get_Home2()==0)
		{
			set_speed(0,0,0);
			set_speed(0,-49,0);
			osDelay(280);
			set_speed(0,0,0);
			break;
		}
	}
		
	Turn_angle(2,-90,0);
	osDelay(2000);	
		
		
	}
	
	
	else
	{
		GO_home = 1;
	set_speed(-90,0,0);//ײ����������̬
	osDelay(1000);
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	set_speed(-12,-40,0);
		osDelay(1000);
		set_speed(0,0,0);
//	while(1)
//	{
//		
//        if(Get_HW_NEW1()==0)
//		{
//			set_speed(0, 0, 0);
//			
//			break;
//		}
//	}
	ActionGroup(10,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
		
	move_by_encoder(2,-620);
	Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
	Turn_angle(2,-90,0);
	osDelay(1500);
	

	
	
	
	move_by_encoder(2,200);
	Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
	Turn_angle(2,-90,0);
//	osDelay(000);
	
int i = 0;
	while(1)
	{
		Start_red:
		set_speed(-90,0,0);
		
		if(Get_Home1()==0)
		{
			set_speed(0,0,0);
			Turn_angle(2,-90,0);
			osDelay(100);
			if(Get_Home1()==1)goto Start_red;
			
			while(1)
			{

			Move_inch(90,0,160);
			Turn_angle(2,-90,0);
				
			if(Get_Home1()==1)
				{
					Move_inch(90,0,220);
					Turn_angle(2,-90,0);
					break;
				}
					
	
			}		
			set_speed(0,0,0);
			break;	
		}
	}
	Turn_angle(2,-90,0);		


		set_speed(0,30,0);
	
	while(1)
	{
		if(Get_Home2()==0)
		{
			set_speed(0,0,0);
			set_speed(0,-50,0);
			osDelay(300);
			set_speed(0,0,0);
			break;
		}
	}
		

	Turn_angle(2,-90,0);
	osDelay(2000);
	
	
	
	}
	set_speed(0,0,0);
}

//��԰����
void run_round(int color)//��׮����
{
	int cnt = 0;
	int cnt_count = 0;
	int time_count = 0;
	unsigned int delay_count = 0;
	
	ActionGroup(81,1);
	
	Move_inch(90,0,1050);//�Ӳֿ��ƶ�����
	
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	set_speed(0,-70,0);
	while(1)
		{
			if(Get_HW_LEFT()==1)
			{
				
				while(1)
				{
					if(Get_HW_LEFT()==1)
					{
						osDelay(200);
						set_speed(0,0,0);
						break;
					}
				}
					break;
			}
		}
		osDelay(500);
//	Turn_angle(1,-90,0);
	
	while(1)
	{
		if(Get_US_Distance()>=Low&&Get_US_Distance()<=High)
		{
			set_speed(0,0,0);
			osDelay(20);
			if(Get_US_Distance()>=Low&&Get_US_Distance()<=High)
			break;
			else
			continue;
		}
		
		else if(Get_US_Distance()<=160)
		{
			set_speed(-60,0,0);
		}
		
		else if(Get_US_Distance()>=170)
		{
			set_speed(60,0,0);
		}
	
	}
	
//	set_speed(0,-70,0);
//	while(1)
//		{
//			if(Get_HW_LEFT()==1)
//			{
//				osDelay(200);
//				set_speed(0,0,0);
//				break;
//			}
//		}
//	
	
	
	int i = 18;

	extern int stake;
	extern int stake_count;
	int round_count  = 0;
		
	for(uint8_t i=0;i<3;i++)
	{
		 MV_SendCmd(1, 0); //ʹ��mv
		 osDelay(50);
		 MV_SendCmd(2, color); //������ɫΪĿ����
		 osDelay(50);
	}
	 Set_QueryState(1);
	Enable_ServoFlag();
	stake =1;
	stake_count = 0;
	ActionGroup(71,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
	
	set_imu_status(0);
	clear_motor_data();
	 Set_FirstFlag(0);
		cnt_count = 3;
	while(1)
	{
		
		Start_round:
		
		if(Get_HW_LEFT()==1&&Get_HW_RIGHT()==1&&(stake_count ==0||cnt_count == 1||cnt_count == 3))
		{
			set_speed(0,-40,43);//һ�������Ǿ�����תȦ��
			
			if(cnt_count == 1)
			{
				delay_count++;
			}
			if(delay_count >= 100000&&cnt_count == 1)
			{
				cnt_count = 2;
				stake_count = 0;
			}
			if(cnt_count == 2)
			{
				if((Get_Yaw()>=-180&&Get_Yaw()<=-176))
				{
					round_count++;
					
				}
			}
			if(round_count>=100000)cnt_count = 3;
			if(cnt_count == 3)
			{
				
				
				if((Get_Yaw()>=-180&&Get_Yaw()<=-176)||(Get_Yaw()>=176&&Get_Yaw()<=180))
				{
					set_speed(0,0,0);
					set_imu_status(1);
					Turn_angle(2,-180,0);
					break;
				}
			}
				
			
		}
		
		else if(Get_HW_LEFT()==0&&Get_HW_RIGHT()==1&&(stake_count ==0||cnt_count == 1||cnt_count == 3))
		{
		while(1)
			{
				if(Get_HW_RIGHT()==0&&Get_HW_LEFT()==1)
				{
					set_speed(0,0,50+4*i);
					osDelay(5);
				}
				else if(Get_HW_LEFT()==0&&Get_HW_RIGHT()==1)
				{
					set_speed(0,0,-30);
					osDelay(5); 
				}
				else
				break;
			}

		}
	
		else if(Get_HW_RIGHT()==0&&Get_HW_LEFT()==1&&(stake_count ==0||cnt_count == 1||cnt_count == 3))
		{
			
			while(1)
			{
				if(Get_HW_RIGHT()==0&&Get_HW_LEFT()==1)
				{
					set_speed(0,-45,50+5*i);
					osDelay(5);
				}
				else if(Get_HW_LEFT()==0&&Get_HW_RIGHT()==1)
				{
					set_speed(0,0,-30);
					osDelay(5); 
				}
				else
				break;
			}
			

		}
		
		else if(Get_HW_RIGHT()==0&&Get_HW_LEFT()==0&&(stake_count ==0||cnt_count == 1||cnt_count == 3))
		{
			set_speed(0,-50,0);
			osDelay(5);
		}
		
		else if(stake_count == 1&&(cnt_count == 0||cnt_count == 2))
		{
			time_count++;
			if(time_count == 1)
			{
				cnt_count = 1;
				stake_count = 0;
				goto Start_round;
			}
			
			ActionGroup(72, 1);
			Wait_Servo_Signal(Wait_Dealy_MAX);
			ActionGroup(71,1);
			Wait_Servo_Signal(Wait_Dealy_MAX);
			stake_count = 0;
			
			cnt++;
			
			if(cnt >= 0)
			{
				cnt_count = 3;
			}
			
		}
		
	
	
	}
	
	set_speed(0,-70,0);
	while(1)
		{
			if(Get_HW_LEFT()==1)
			{
				set_speed(0,0,0);
					break;
			}
		}
	
	while(1)
	{
		if(Get_US_Distance()>=168&&Get_US_Distance()<=178)
		{
			set_speed(0,0,0);
			osDelay(5);
			if(Get_US_Distance()>=168&&Get_US_Distance()<=178)
			break;
			else
			continue;
		}
		
		else if(Get_US_Distance()<=168)
		{
			set_speed(-90,0,0);
		}
		
		else if(Get_US_Distance()>=178)
		{
			set_speed(90,0,0);
		}
	
	}
	Turn_angle(2,-180,0);
	set_imu_status(0);
	ActionGroup(73,1);
	Wait_Servo_Signal(300000);
	set_imu_status(1);
	

	 MV_SendCmd(0, 0);
	Set_QueryState(0);
}
	

	
	
	



void blue_all(void)
{

	Move_inch(230,200,2000);
	
	
	
	
	set_speed(0,200,0);
	osDelay(500);
	ActionGroup(81,1);
//	Wait_Servo_Signal(Wait_Dealy_MAX);
	osDelay(3600);
	set_speed(0,0,0);
	
	
	while(Get_HW(2)==0)
	{
		set_speed(0,100,0);
	
	}
	set_speed(0,0,0);
		
	set_speed(120,0,0);
	osDelay(1000);
	
	 for(uint8_t i=0;i<3;i++)
		{
		 
		 Set_InitYaw(0);
		 osDelay(50);
		
		}
  
//	while(Get_HW(2)==1)
//	{
//		set_speed(0,-100,0);
//	
//	}
//	set_speed(0,0,0);

//	move_by_encoder(2,100);//��Բ�̻�����
//	Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);	
	
	 ActionGroup(82, 1); //���۵�״̬�����е��
    Wait_Servo_Signal(Wait_Dealy_MAX);
	ActionGroup(2, 1); //�ı�洢����
    Wait_Servo_Signal(Wait_Dealy_MAX);
	
	
//	centren_control();//��λ��Բ������
//	
//	

////	control_ball(2);//ץ��
//	Dump_ball();
//	
//	run_ladder();
	
//	set_imu_param(102,0,0);
//	ActionGroup(53,1);
//	Wait_Servo_Signal(Wait_Dealy_MAX);
//	Ladder_process(1,1,0);//��ʼ��һ�ν���ƽ̨
//	
//	Restart_Ladder();

//	set_imu_param(82,32,70);





//	Dump_RoundRing();
//	Dump_block();

	

//	

}

void run_blue(void)
{
	
	
	color = 0;
	//·���滮

	centren_control(0);//��λ��Բ�����Ĳ��ұ���
	
	control_ball(2);//ץ��
//	
	Dump_ball(0);//����
//	
	run_round(2);
	
	run_ladder(0);
	
	Dump_RoundRing(0);//�����
	
	Dump_block();//�Ż�ľ��
	
	run_home(0);
	osDelay(5000);
	set_speed(0,0,0);
	vTaskDelete(NULL);
}


void run_red(void)
{
	color = 1;
	

	//·���滮

	centren_control(1);//��λ��Բ�����Ĳ��ұ���
	
	control_ball(0);//ץ��
	
	Dump_ball(1);//����
	
	run_round(0);
	
	run_ladder(1);
	
	Dump_RoundRing(1);//�����
	
	Dump_block();//�Ż�ľ��
	
	//������ȫ�ֲ��־��������ˣ�15��֮��Ҫ����׮
	
	run_home(1);
	
    set_speed(0,0,0);
	osDelay(5000);
	 vTaskDelete(NULL);
}
