#include "Odomerter.h"
#include "motor.h"
#include "math.h"
#include "pid.h"
#include "board.h"
#define PI 3.1415926
//里程计值
float Odom_Angl = 0.0,Odom_X = 0.0,Odom_Y = 0.0,LSpeed=0,RSpeed=0;
long Total_Now_TwoPulse=0;                                                //当前的脉冲
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
  * @brief  浮点数获取绝对值
  * @param  输入值
  * @retval 绝对值
	*/
int int_abs(int in)
{
	int out;
	out = in > 0 ? in : -in;
	return out;
}


/**
  * @brief  浮点数获取绝对值
  * @param  输入值
  * @retval 绝对值
	*/
float float_abs(float in)
{
	float out;
	out = in > 0 ? in : -in;
	return out;
}

/**
  * @brief  距离限幅，判断小车是否走的距离够数
  * @param1  当前距离
  * @param2  目标距离
  * @retval 返回标志位
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
  * @brief  获取小车角度
  * @param1 左轮速度
	* @param2 右轮速度
  * @retval None
	*/
void Odom_Get(float LSpeed,float RSpeed)
{
	//单位量10MS
	Odom_Omega = (RSpeed - LSpeed) / 100.0 / 13 * 180 / PI;		 //角速度：速度差/轮距(13CM)*弧度转角度
	Car_Speed = (RSpeed + LSpeed) / 2 / 100.0;
	Odom_X += cos(Odom_Theta * PI / 180) * Car_Speed;
	Odom_Y += sin(Odom_Theta * PI / 180) * Car_Speed;
	//Odom_Theta += Odom_Omega ;				//积分出角度
//	if(Odom_Theta <- 89||Odom_Theta > 89)
//	{
//		flag++;
//		if(flag%2==1)Odom_Theta = 0;
//	}
}

/**
  * @brief  使小车转向
  * @param1 实际角度
	* @param2 目标角度
  * @retval None
	*/
void Odom_Turn(float Act_Ang,float Tar_Ang)
{
	float Delta_Ang = 0.0;				 //角度增量
	Delta_Ang = Act_Ang-Tar_Ang;	//角度变化值
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
  * @brief  任务一：任意两点直行，伴有声光提示
  * @param  θ角
  * @retval None
	*/


/**
  * @brief  任务三：对角线--弧线--对角线--弧线
  * @param  θ角
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
			if(DL_GPIO_readPins(track_pin_9_PORT,track_pin_9_PIN) > 0){Task3_Flag = 2;flagf=1; }//直线到循迹
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
  * @brief  任务一：任意两点直行，伴有声光提示
  * @param  θ角
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

