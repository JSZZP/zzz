#include "board.h"
#include "tim.h"
#include "uart.h"
#include "motor.h"
#include "Odomerter.h"
#include "stdio.h"

#define u16 uint16_t
#define PI 3.1415926 

extern u16 tt;
extern PID Track;
extern int LPWM, RPWM;
extern int track_data;
extern float LSpeed,RSpeed;
extern long int Total_Now_TwoPulse;
extern PID LMotor,RMotor,Angle;
extern float Odom_Angl,Odom_X ,Odom_Y;
extern float Odom_Omega,Odom_Theta,Car_Speed;
float Count1,Count2;
extern int flagf;
extern double angular_z;
u8 connect = 0;
u8 stop_flag = 0;
u8 buf[64];
u16 ttt;
char buff[15];
extern u8 Task2_Flag,stop_Count;
//int flag=0;
void f2str(char *buf, float num) {
    // 处理负数
    if (num < 0) {
        *buf++ = '-';
        num = -num;
    }

    // 获取整数部分
    int int_part = (int)num;
    // 获取小数部分并保留两位小数
    int frac_part = (int)((num - int_part) * 100);

    // 处理小数部分负数
    if (frac_part < 0) {
        frac_part = -frac_part;
    }
    // 将整数部分转换为字符串
    sprintf(buf, "%d.%02d", int_part, frac_part);
}

	
//void TIMER_0_INST_IRQHandler()//1ms
//{
//	switch(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
//	{
//		case DL_TIMERA_IIDX_ZERO:
//				if(++ tt == 20)
//				{
//					tt=0;
//					Feed_Back();
//					if( track_data == 0 ){stop_flag = 0; Work_Mode(0,0); LPWM = 300; RPWM = 300;}
//					else if ( track_data == -24 ){stop_flag = 0; Work_Mode(0,0); LPWM = 100; RPWM = 300;}
//					else if ( track_data == -16 ){stop_flag = 0; Work_Mode(0,0); LPWM = 200; RPWM = 300;}
//					else if ( track_data == -8 ){stop_flag = 0; Work_Mode(0,0); LPWM = 200; RPWM = 350;}
//					else if ( track_data == 8 ){stop_flag = 0; Work_Mode(0,0); LPWM = 350; RPWM = 200;}
//					else if ( track_data == 16 ){stop_flag = 0; Work_Mode(0,0); LPWM = 300; RPWM = 200;}
//					else if ( track_data == 24 ){stop_flag = 0; Work_Mode(0,0); LPWM = 300; RPWM = 100;}
//					else if ( track_data == 100 ) {if(++stop_flag >= 5) {stop_flag = 0;Work_Mode(2,2); LPWM = 0; RPWM = 0; DL_GPIO_setPins(LED1_PORT,LED1_PIN_14_PIN);} }

//					
//					
//					Motor(&LPWM,&RPWM);
//					
//					//Motor(&LPWM,&RPWM);
//				}
//			break;
//		default:
//			break;
//	}
//}

void SysTick_Handler(void)//1ms
{
//		printf("%f\r\n",angular_z);
	//Odom_Theta=angular_z;
	if(flagf==1)
	{
		DL_GPIO_setPins(fmq_PORT,fmq_PIN_fmq_PIN);
//		flagf=0;
	}
		if(flagf==0)
	{
		DL_GPIO_clearPins(fmq_PORT,fmq_PIN_fmq_PIN);
//		flagf=0;
	}
		Total_Now_TwoPulse+= int_abs(Count1) + int_abs(Count2);
		
		LSpeed = (Count1) * PI * 4.8 / 5.2;           //左右实际速度，R/s*2Pi*R = V
		RSpeed = (Count2) * PI * 4.8 / 5.2;
		
		Odom_Get(LSpeed,RSpeed);                     //通过积分获取角度
//	if(Odom_Theta <- 90||Odom_Theta > 90)
//		{
//			flag++;
//			if(flag%2==1)Odom_Theta = 0;
//		}

		if(stop_flag == 1)stop_Count++;           //对mian函数的stepflag进行计数
		
//		LPWM = 0 + PID_Speed(&LMotor,LSpeed);
//		RPWM = 0 + PID_Speed(&RMotor,RSpeed);
		
		
/***************测试区************/
		//Task3(Odom_Theta);	
		
		
		
		//装载PWM
		Motor(&LPWM,&RPWM);                       
		
		Count1=0;                                     //编码器计数值清零
		Count2=0;
//		if(++ttt==100){DL_GPIO_togglePins(LED1_PORT,LED1_PIN_14_PIN);ttt=0;
//			sprintf(buff,"%d",Task2_Flag);
//			UART_1_printf(buff);}

		
	
}

void GROUP1_IRQHandler(void)//EXTI
{
	uint32_t gpio;
	gpio = DL_GPIO_getEnabledInterruptStatus(Encoder_PORT, Encoder_PIN_18_PIN|Encoder_PIN_20_PIN);
	if( (gpio & Encoder_PIN_18_PIN) == Encoder_PIN_18_PIN )
	{
				if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_18_PIN) > 0)
				{
					if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_19_PIN) > 0)Count1--;//上升沿且B相为正
					if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_19_PIN) <= 0)Count1++;//上升沿且B相为正
				}
				else
				{
					if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_19_PIN) > 0)Count1++;//上升沿且B相为正
					if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_19_PIN) <= 0)Count1--;//上升沿且B相为正
				}
	}
	else if( (gpio & Encoder_PIN_20_PIN) == Encoder_PIN_20_PIN )
	{
				if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_20_PIN) > 0)
				{
					if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_24_PIN) > 0)Count2++;//上升沿且B相为正
					if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_24_PIN) <= 0)Count2--;//上升沿且B相为正
				}
				else
				{
					if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_24_PIN) > 0)Count2--;//上升沿且B相为正
					if(DL_GPIO_readPins(Encoder_PORT, Encoder_PIN_24_PIN) <= 0)Count2++;//上升沿且B相为正
				}
	}
	DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_PIN_18_PIN|Encoder_PIN_20_PIN);
}