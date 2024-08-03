#include "Odomerter.h"
#include "motor.h"
#include "math.h"
#include "pid.h"
#include "board.h"
#define PI 3.1415926
//��̼�ֵ
float Odom_Angl = 0.0,Odom_X = 0.0,Odom_Y = 0.0,LSpeed=0,RSpeed=0;
long Total_Now_TwoPulse=0;                                                //��ǰ������
float Odom_Omega = 0.0,Odom_Theta = 0.0,Car_Speed = 0.0;
extern int LPWM,RPWM;
int flag=0;
char Buff[10];
extern u8 Task3_Flag, Cycle_Flag;
extern int track_data;
extern int flagf;
//void PC13_init()
//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC,&GPIO_InitStructure);
//	
//}

/**
  * @brief  ��������ȡ����ֵ
  * @param  ����ֵ
  * @retval ����ֵ
	*/
int int_abs(int in)
{
	int out;
	out = in > 0 ? in : -in;
	return out;
}


/**
  * @brief  ��������ȡ����ֵ
  * @param  ����ֵ
  * @retval ����ֵ
	*/
float float_abs(float in)
{
	float out;
	out = in > 0 ? in : -in;
	return out;
}

/**
  * @brief  �����޷����ж�С���Ƿ��ߵľ��빻��
  * @param1  ��ǰ����
  * @param2  Ŀ�����
  * @retval ���ر�־λ
  */
int Laps_limit(float Now_Laps,float Target_Laps)
{
	if(Now_Laps > Target_Laps)
	{
		stop();
		return 1;
	}
	return 0;
}


/**
  * @brief  ��ȡС���Ƕ�
  * @param1 �����ٶ�
	* @param2 �����ٶ�
  * @retval None
	*/
void Odom_Get(float LSpeed,float RSpeed)
{
	//��λ��10MS
	Odom_Omega = (RSpeed - LSpeed) / 100.0 / 13 * 180 / PI;		 //���ٶȣ��ٶȲ�/�־�(13CM)*����ת�Ƕ�
	Car_Speed = (RSpeed + LSpeed) / 2 / 100.0;
	Odom_X += cos(Odom_Theta * PI / 180) * Car_Speed;
	Odom_Y += sin(Odom_Theta * PI / 180) * Car_Speed;
	//Odom_Theta += Odom_Omega ;				//���ֳ��Ƕ�
//	if(Odom_Theta <- 89||Odom_Theta > 89)
//	{
//		flag++;
//		if(flag%2==1)Odom_Theta = 0;
//	}
}

/**
  * @brief  ʹС��ת��
  * @param1 ʵ�ʽǶ�
	* @param2 Ŀ��Ƕ�
  * @retval None
	*/
void Odom_Turn(float Act_Ang,float Tar_Ang)
{
	float Delta_Ang = 0.0;				 //�Ƕ�����
	Delta_Ang = Act_Ang-Tar_Ang;	//�Ƕȱ仯ֵ
	if(float_abs(Act_Ang-Tar_Ang) > 1 )
	{
		LPWM = RPWM = 100 ;
		Delta_Ang < 0 ? left(): right();
		//Motor(&LPWM,&RPWM);
	}
	else
	{
		LPWM = RPWM = 0 ;
		stop();
	}
	
}

/**
  * @brief  ����һ����������ֱ�У�����������ʾ
  * @param  �Ƚ�
  * @retval None
	*/


/**
  * @brief  ���������Խ���--����--�Խ���--����
  * @param  �Ƚ�
  * @retval None
	*/
void Task3(float Act_Ang,int TAR_ANG,int dis)
{
	
	if(float_abs(Act_Ang-TAR_ANG) > 2 && Task3_Flag == 0)
	{	
		
		LPWM = RPWM = 105 ;
		Act_Ang-TAR_ANG  < 0 ? left(): right();
	}
	else
	{
		if(Task3_Flag == 0)
		{
			Angle.Target=TAR_ANG;
			stop();
			Task3_Flag = 1;
			Odom_X=0;
		}
		else if(Task3_Flag == 1)
		{
			LPWM = RPWM = 270 ;
			if(float_abs(Act_Ang) > 1 )
			{
				LPWM = 270 - PID_Angle(&Angle,Act_Ang);
				RPWM = 270 + PID_Angle(&Angle,Act_Ang);
			}
			if(float_abs(Odom_X) > dis){Task3_Flag = 2;flagf=1;  }
			if(DL_GPIO_readPins(track_pin_9_PORT,track_pin_9_PIN) > 0){Task3_Flag = 2;flagf=1; }//ֱ�ߵ�ѭ��
			if(DL_GPIO_readPins(track_pin_22_PORT,track_pin_22_PIN) > 0){Task3_Flag = 2;flagf=1; }
			if(DL_GPIO_readPins(track_pin_23_PORT,track_pin_23_PIN) > 0){Task3_Flag = 2; flagf=1;}
			if(DL_GPIO_readPins(track_pin_24_PORT,track_pin_24_PIN) > 0){Task3_Flag = 2; flagf=1;}
			if(DL_GPIO_readPins(track_pin_27_PORT,track_pin_27_PIN) > 0){Task3_Flag = 2; flagf=1;}

			else {stright();}
		}
		else if(Task3_Flag == 2)
		{
			if(float_abs(Act_Ang - TAR_ANG) > 2)
			{
				LPWM = RPWM = 105 ;
				Act_Ang-TAR_ANG < 0 ? left(): right();
			}
			else 
			{
				flagf=0;
				stop();
				Task3_Flag=3; Cycle_Flag = 1;
			}
		}
	}
}
/**
  * @brief  ����һ����������ֱ�У�����������ʾ
  * @param  �Ƚ�
  * @retval None
	*/
void Task1(float Act_Ang,float Act_Omega)
{
	static u8 Task1_Flag = 0;

	if(Task1_Flag == 0)
	{
		delay_ms(2000);
		LPWM = RPWM = 400;
		Task1_Flag = 1;
	}
	else if(Task1_Flag == 1)
	{
		if(float_abs(Act_Ang) > 1 )
		{
			LPWM = 400 - PID_Angle(&Angle,Act_Ang);
			RPWM = 400 + PID_Angle(&Angle,Act_Ang);
			
		}
		if(Odom_X < 100) stright();
		else
		{
			Odom_Turn(Odom_Theta,0);
		}
		//Task1_Flag = 0;
	}
}

