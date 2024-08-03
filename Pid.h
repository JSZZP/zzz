#ifndef __PID_H
#define __PID_H


typedef struct
{
	float Target,Actual;
	float Kp,Ki,Kd;
	float Error_0,Error_1,Error_2;
	float Error_Last;
	float Error_Sum;
	float Output;	 // ���
	float MaxIntegral,MaxOutput; // PID��� �����޷�������޷�
}PID;

void PID_Init(void);
float PID_Speed(PID *pid,float Actual_Val);
float PID_Angle(PID *pid,float Actual_Val);

#endif

