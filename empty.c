/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "board.h"
#include <stdio.h>
#include "inv_mpu.h"
#include "Odomerter.h"
#define u16 uint16_t
#define u8 uint8_t
extern int error;
extern long int Total_Now_TwoPulse;
extern float Odom_Angl,Odom_X ,Odom_Y;
extern float Odom_Omega,Odom_Theta,Car_Speed;
extern int track_data;
extern u8 stop_flag;
float a1,a2,a3,k;
int flagf=0;
//int yaw;
u16 tt;
PID Track;
int LPWM = 0, RPWM = 0;
extern float Count1, Count2;
extern u8 connect;
u8 Task3_Flag = 0 ,Cycle_Flag = 0;
u8 select = 0;
u8 Task2_Flag = 1,stop_Count = 0;
int choose=3;
uint16_t RxBuffer1[11] = {0};
double angular_z;
void xunji()
{
	Feed_Back();
	if( track_data == 0 ){stop_flag = 0; stright();  LPWM = 200; RPWM = 200;}
	else if ( track_data == -24 ){stop_flag = 0; left(); LPWM = 400; RPWM = 400;}
	else if ( track_data == -16 ){stop_flag = 0; stright();LPWM = 100; RPWM = 250;}
	else if ( track_data == -8 ){stop_flag = 0;  stright();LPWM = 80; RPWM = 250;}
	else if ( track_data == 8 ){stop_flag = 0;  stright();LPWM = 250; RPWM = 80;}
	else if ( track_data == 16 ){stop_flag = 0;  stright();LPWM = 250; RPWM = 100;}
	else if ( track_data == 24 ){stop_flag = 0; right(); LPWM = 400; RPWM = 400;}
	//else if ( track_data == 100 ) {if(++stop_flag >= 25)stop_flag = 25;}
	else if ( track_data == 100 ) stop_flag = 1;
	else if ( track_data == 120 ) stop_flag = 1;
	//Motor(&LPWM,&RPWM);
}

void Task2(float Act_Ang)
{
	
	if(Task2_Flag == 1)//第一步直行
	{
		LPWM = RPWM = 350 ;
		if(float_abs(Act_Ang) > 1 )
		{
			LPWM = 350 - PID_Angle(&Angle,Act_Ang);
			RPWM = 350 + PID_Angle(&Angle,Act_Ang);
		}
		if(Odom_X > 100)Task2_Flag = 2;
		else stright();
	}
	else if(Task2_Flag == 2)//位置修正并停止2s
	{
		//stop();LPWM=0;RPWM=0;
		if(float_abs(Act_Ang - 0) > 1)
		{
			LPWM = RPWM = 150 ;
			Act_Ang - 0 < 1 ? left(): right();
		}
		else 
		{
			stop();LPWM=0;RPWM=0;
			//delay_ms(2000);
			Task2_Flag=3;
		}
	}
	else if(Task2_Flag == 3)//开始循迹
	{
		xunji();
		if(stop_flag == 5)
		{
			stop();
			Odom_X = 0;Odom_Theta = 0;
			Task2_Flag = 4;
		}
		
	}
}

int main(void)
{
      //开发板初始化
      board_init();
	  delay_ms(2000);
      while(1)
      {
				if(choose==1)
				{
						  Task3(Odom_Theta,0,100);
				}
				if(choose==2)
				{
					if(select==0)
					{
						Task3(Odom_Theta,0,200);
						if(Task3_Flag==3)
						{
						 // UART_1_printf("complete\r\n");
							xunji();
		//				  if(stop_flag==25) { LPWM = RPWM = 0;  delay_ms(500); stop_flag = 0; select = 1;  Task3_Flag = 0;}
							if(stop_Count > 50){ stop(); stop_flag = 0; select = 1;  Task3_Flag = 0;}
							if(stop_flag == 0)stop_Count=0;
						}
					}
					
					if(select==1)
					{	 
						if(Odom_Y<-76)
						{
							a1=0.08*(Odom_Y)-175;
						Task3(Odom_Theta,a1,200);
						}
							if(Odom_Y==-80)
						{
						Task3(Odom_Theta,-173,200);
						}
						if(Odom_Y>84)
						{
							Task3(Odom_Theta,-170,200);
						}
						if(Task3_Flag==3)
						{
							xunji();
							if(stop_Count > 50){ stop(); stop_flag = 0;  select = 2;  Task3_Flag = 0;flagf=1;}
							if(stop_flag == 0)stop_Count=0;
						}
					}				
				}
				if(choose==3)
				{
					if(select==0)
					{
						Task3(angular_z,-34,1000);
						if(Task3_Flag==3)
						{
						 // UART_1_printf("complete\r\n");
							xunji();
		//				  if(stop_flag==25) { LPWM = RPWM = 0;  delay_ms(500); stop_flag = 0; select = 1;  Task3_Flag = 0;}
							if(stop_Count > 50){ stop(); stop_flag = 0; select = 1;  Task3_Flag = 0;flagf=1;}//
							if(stop_flag == 0)stop_Count=0;
						}
					}
					
					if(select==1)
					{	 
						Task3(angular_z,-140,1000);
						if(Task3_Flag==3)
						{
							xunji();
							if(stop_Count > 50){ stop(); stop_flag = 0;  select = 0;  Task3_Flag = 0;flagf=1;}
							if(stop_flag == 0)stop_Count=0;
						}
					}
			}					
	}
//		  Task2(Odom_Theta);
//		  Motor(&LPWM,&RPWM);
//		  if (Cycle_Flag == 2)
//		  {
//			  Work_Mode(0,0);
//			  //Odom_Theta=0;
//			  Task3(Odom_Theta,0,100);
//		  }
		  //Task3(Odom_Theta);	
		 // Odom_Turn(Odom_Theta,90);
		  //printf("%.2f",Odom_X);
		  //delay_ms(10);
//		  Task1(Odom_Theta,0);
		  // delay_ms(1000);
////		  if(select==0)
////		  {
////			  Task3(Odom_Theta,0,100);
////			  if(Task3_Flag==3)
////			  {
////				 // UART_1_printf("complete\r\n");
////				  xunji();
//////				  if(stop_flag==25) { LPWM = RPWM = 0;  delay_ms(500); stop_flag = 0; select = 1;  Task3_Flag = 0;}
////				  if(stop_Count > 50){ stop(); stop_flag = 0; select = 1;  Task3_Flag = 0;}
////				  if(stop_flag == 0)stop_Count=0;
////			  }
////		  }
////		  
////		  if(select==1)
////		  {	 
////			  Task3(Odom_Theta,-175,80);
////			  if(Task3_Flag==3)
////			  {
////				  xunji();
////				  if(stop_Count > 50){ stop(); stop_flag = 0;  select = 2;  Task3_Flag = 0;}
////				  if(stop_flag == 0)stop_Count=0;
////			  }
////		  }
//		  if(select==0)
//		  {
//			  Task3(Odom_Theta,-29,140);
//			  if(Task3_Flag==3)
//			  {
//				 // UART_1_printf("complete\r\n");
//				  xunji();
////				  if(stop_flag==25) { LPWM = RPWM = 0;  delay_ms(500); stop_flag = 0; select = 1;  Task3_Flag = 0;}
//				  if(stop_Count > 50){ stop(); stop_flag = 0; select = 1;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//		  
//		  if(select==1)
//		  {	 
//			  Task3(Odom_Theta,208,140);
//			  if(Task3_Flag==3)
//			  {
//				  xunji();
//				  if(stop_Count > 50){ stop(); stop_flag = 0;  select = 2;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//			if(select == 2)
//			{
//				select = 0;
//			}


//		  if(select==0)
//		  {
//			  Task3(Odom_Theta,-29,140);
//			  if(Task3_Flag==3)
//			  {
//				 // UART_1_printf("complete\r\n");
//				  xunji();
////				  if(stop_flag==25) { LPWM = RPWM = 0;  delay_ms(500); stop_flag = 0; select = 1;  Task3_Flag = 0;}
//				  if(stop_Count > 50){ stop(); stop_flag = 0; select = 1;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//		  
//		  if(select==1)
//		  {	 
//			  Task3(Odom_Theta,208,140);
//			  if(Task3_Flag==3)
//			  {
//				  xunji();
//				  if(stop_Count > 50){ stop(); stop_flag = 0;  select = 2;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//		  if(select==2)
//		  {
//			  Task3(Odom_Theta,-33,140);
//			  if(Task3_Flag==3)
//			  {
//				 // UART_1_printf("complete\r\n");
//				  xunji();
////				  if(stop_flag==25) { LPWM = RPWM = 0;  delay_ms(500); stop_flag = 0; select = 1;  Task3_Flag = 0;}
//				  if(stop_Count > 50){ stop(); stop_flag = 0; select = 3;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//		  
//		  if(select==3)
//		  {	 
//			  Task3(Odom_Theta,200,140);
//			  if(Task3_Flag==3)
//			  {
//				  xunji();
//				  if(stop_Count > 50){ stop(); stop_flag = 0;  select = 4;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//		  if(select==4)
//		  {
//			  Task3(Odom_Theta,-29,140);
//			  if(Task3_Flag==3)
//			  {
//				 // UART_1_printf("complete\r\n");
//				  xunji();
////				  if(stop_flag==25) { LPWM = RPWM = 0;  delay_ms(500); stop_flag = 0; select = 1;  Task3_Flag = 0;}
//				  if(stop_Count > 50){ stop(); stop_flag = 0; select = 5;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//		  
//		  if(select==5)
//		  {	 
//			  Task3(Odom_Theta,208,140);
//			  if(Task3_Flag==3)
//			  {
//				  xunji();
//				  if(stop_Count > 50){ stop(); stop_flag = 0;  select = 6;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//		  if(select==6)
//		  {
//			  Task3(Odom_Theta,-29,140);
//			  if(Task3_Flag==3)
//			  {
//				 // UART_1_printf("complete\r\n");
//				  xunji();
////				  if(stop_flag==25) { LPWM = RPWM = 0;  delay_ms(500); stop_flag = 0; select = 1;  Task3_Flag = 0;}
//				  if(stop_Count > 50){ stop(); stop_flag = 0; select = 7;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }
//		  
//		  if(select==7)
//		  {	 
//			  Task3(Odom_Theta,208,140);
//			  if(Task3_Flag==3)
//			  {
//				  xunji();
//				  if(stop_Count > 50){ stop(); stop_flag = 0;  select = 8;  Task3_Flag = 0;}
//				  if(stop_flag == 0)stop_Count=0;
//			  }
//		  }

}




