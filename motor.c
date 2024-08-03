#include "motor.h"
#include "stdio.h"
#include "board.h"
#include "stdlib.h"
#include "Odomerter.h"

#define PWM_MAX 		500
#define PWM_MIN			0

#define R2 DL_GPIO_readPins(track_pin_9_PORT,track_pin_9_PIN)
#define R1 DL_GPIO_readPins(track_pin_22_PORT,track_pin_22_PIN)
#define MID DL_GPIO_readPins(track_pin_23_PORT,track_pin_23_PIN)
#define L1 DL_GPIO_readPins(track_pin_24_PORT,track_pin_24_PIN)
#define L2 DL_GPIO_readPins(track_pin_27_PORT,track_pin_27_PIN)

uint32_t T[5];
int track_data;
extern int LPWM,RPWM;
extern PID Track;
extern float Odom_Theta;
	
void Work_Mode(u8 Lmode, u8 Rmode)
{
	if(Lmode == 0) 		//前进
	{
		DL_GPIO_setPins(MODEL_PORT, MODEL_PIN_8_PIN);
		DL_GPIO_clearPins(MODEL_PORT, MODEL_PIN_9_PIN);
	}
	else if(Lmode == 1)			//后退
	{
		DL_GPIO_clearPins(MODEL_PORT, MODEL_PIN_8_PIN);
		DL_GPIO_setPins(MODEL_PORT, MODEL_PIN_9_PIN);
	}
	else if(Lmode ==2)	//停止
	{
		DL_GPIO_clearPins(MODEL_PORT, MODEL_PIN_8_PIN);
		DL_GPIO_clearPins(MODEL_PORT, MODEL_PIN_9_PIN);
	}
	
	if(Rmode == 0) 		//前进
	{
		DL_GPIO_setPins(MODER_PORT, MODER_PIN_7_PIN);
		DL_GPIO_clearPins(MODER_PORT, MODER_PIN_6_PIN);
	}
	else if(Rmode == 1)			//后退
	{
		DL_GPIO_clearPins(MODER_PORT, MODER_PIN_7_PIN);
		DL_GPIO_setPins(MODER_PORT, MODER_PIN_6_PIN);
	}
	else if(Rmode ==2)		//停止
	{
		DL_GPIO_clearPins(MODER_PORT, MODER_PIN_7_PIN);
		DL_GPIO_clearPins(MODER_PORT, MODER_PIN_6_PIN);
	}
	
}

void stright()
{
	Work_Mode(0,0);
}

void left()
{
	Work_Mode(1,0);
}

void right()
{
	Work_Mode(0,1);
}

void stop()
{
	Work_Mode(2,2);
}

void back()
{
	Work_Mode(1,1);
}

void Motor(int * Lmotor, int * Rmotor)
{
	
	//if( *Lmotor > 0 )
	//{
		if( *Lmotor >= PWM_MAX ) *Lmotor = PWM_MAX;
		if( *Lmotor <= PWM_MIN ) *Lmotor = PWM_MIN;
	//}
	//if( *Rmotor > 0 )
	//{
		if( *Rmotor >= PWM_MAX ) *Rmotor = PWM_MAX;
		if( *Rmotor <= PWM_MIN ) *Rmotor = PWM_MIN;
	//}
//	if ( *Lmotor > 0 )
//	{
//		DL_GPIO_setPins(MODEL_PORT, MODEL_PIN_8_PIN);
//		DL_GPIO_clearPins(MODEL_PORT, MODEL_PIN_9_PIN);
//		DL_TimerA_setCaptureCompareValue(PWM_Wheel_INST, *Lmotor, GPIO_PWM_Wheel_C0_IDX);
//	}//PA17;
//	if ( *Lmotor < 0 ) 
//	{
//		DL_GPIO_clearPins(MODEL_PORT, MODEL_PIN_8_PIN);
//		DL_GPIO_setPins(MODEL_PORT, MODEL_PIN_9_PIN);
//		DL_TimerA_setCaptureCompareValue(PWM_Wheel_INST, abs(*Lmotor), GPIO_PWM_Wheel_C0_IDX);
//	}
//	if ( *Rmotor > 0 )
//	{
//		DL_GPIO_setPins(MODER_PORT, MODER_PIN_7_PIN);
//		DL_GPIO_clearPins(MODER_PORT, MODER_PIN_6_PIN);
//		DL_TimerA_setCaptureCompareValue(PWM_Wheel_INST, *Rmotor, GPIO_PWM_Wheel_C1_IDX);
//	}//PA17;
//	if ( *Rmotor < 0 ) 
//	{
//		DL_GPIO_clearPins(MODER_PORT, MODER_PIN_7_PIN);
//		DL_GPIO_setPins(MODER_PORT, MODER_PIN_6_PIN);
//		DL_TimerA_setCaptureCompareValue(PWM_Wheel_INST, abs(*Rmotor), GPIO_PWM_Wheel_C1_IDX);
//	}
		DL_TimerA_setCaptureCompareValue(PWM_Wheel_INST, *Lmotor, GPIO_PWM_Wheel_C0_IDX);
		DL_TimerA_setCaptureCompareValue(PWM_Wheel_INST, *Rmotor, GPIO_PWM_Wheel_C1_IDX);
}
	
void Feed_Back()
{
	//if(Odom_Theta<=-170){	track_data = 120;}

	 if( L2 <= 0 && L1 <= 0 && MID > 0 && R1 <= 0 && R2 <= 0 ){track_data = 0;LPWM = 250; RPWM = 250;}//00100
	else if	( L2 > 0&& L1 <= 0 && MID <= 0 && R1 <= 0 && R2 <= 0 )track_data = -24;//10000
	else if	( L2 <= 0 && L1 > 0 && MID <= 0 && R1 <= 0 && R2 <= 0 )track_data = -8;//01000
	else if	( L2 <= 0 && L1 <= 0 && MID <= 0 && R1 > 0 && R2 <= 0 )track_data = 8;//00010
	else if	( L2 <=0 && L1 <= 0 && MID <= 0 && R1 <= 0 && R2 > 0 )track_data = 24;//00001
	
	else if	( L2 > 0&& L1 <= 0 && MID <= 0 && R1 <= 0 && R2 <= 0 )track_data = -16;//11000
	else if	( L2 <= 0 && L1 > 0 && MID <= 0 && R1 <= 0 && R2 <= 0 )track_data = -8;//01100
	else if	( L2 <= 0 && L1 <= 0 && MID <= 0 && R1 > 0 && R2 <= 0 )track_data = 8;//00110
	else if	( L2 <=0 && L1 <= 0 && MID <= 0 && R1 <= 0 && R2 > 0 )track_data = 16;//00011
	
	else if ( L2 <=0 && L1 <= 0 && MID <= 0 && R1 <= 0 && R2 <= 0 )track_data = 100;//00000
//			 if	( track_data!=100&&track_data!=120&&track_data!=0 )track_data =140 ;//11000

}
