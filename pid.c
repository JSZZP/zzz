#include "pid.h"
#include "stdio.h"

extern char Buff[];

PID LMotor,RMotor,Angle;
void PID_Init()
{
	LMotor.Actual = 20.0;
	LMotor.Target = 20.0;
	LMotor.Kp = 0.0;
	LMotor.Ki = 5.0;
	LMotor.Kd = 0.0;
	LMotor.Error_0 = 0.0;
	LMotor.Error_1 = 0.0;
	LMotor.Error_2 = 0.0;
	LMotor.Error_Last = 0.0;
	LMotor.Error_Sum = 0.0;
	LMotor.Output = 0.0;
	LMotor.MaxIntegral	=	600.0;
	LMotor.MaxOutput	=	600.0;
	
	RMotor.Actual = 20.0;
	RMotor.Target = 20.0;
	RMotor.Kp = 0.0;
	RMotor.Ki = 5.0;
	RMotor.Kd = 0.0;
	RMotor.Error_0 = 0.0;
	RMotor.Error_1 = 0.0;
	RMotor.Error_2 = 0.0;
	RMotor.Error_Last = 0.0;
	RMotor.Error_Sum = 0.0;
	RMotor.Output = 0.0;
	RMotor.MaxIntegral	=	600.0;
	RMotor.MaxOutput	=	600.0;
	
	Angle.Actual = 0.0;
	Angle.Target = 0.0;
	Angle.Kp = 5.0;
	Angle.Ki = 0;
	Angle.Kd = 3;
//	Angle.Kp = 20.0;
//	Angle.Ki = 0;
//	Angle.Kd = 10.0;
	Angle.Error_0 = 0.0;
	Angle.Error_1 = 0.0;
	Angle.Error_2 = 0.0;
	Angle.Error_Last = 0.0;
	Angle.Error_Sum = 0.0;
	Angle.Output = 0.0;
	Angle.MaxIntegral	=	20.0;
	Angle.MaxOutput	=	0.0;
}

float PID_Speed(PID *pid,float Actual)
{
	float Pout;
	// DPI
 	// 更新数据，计算偏差
  pid -> Error_0 = pid -> Target - pid -> Actual;	// e(k)
	
	// 计算微分
	float pout = pid->Kp * (pid->Error_0 - pid->Error_1);

	// 计算比例
	float iout = pid->Ki * pid->Error_0;
//	sprintf(Buff,"iout:%.2f ",iout);
//	OLED_ShowString(7,1,Buff,8);
	// 计算积分
  float dout = pid->Kd * (pid->Error_0 - 2 * pid->Error_1 + pid->Error_2);

	//积分限幅
	pid -> Error_Sum = pid -> Error_Sum < pid -> MaxIntegral ? pid -> Error_Sum : pid -> MaxIntegral;
	pid -> Error_Sum = pid -> Error_Sum > -pid -> MaxIntegral ? pid -> Error_Sum : -pid -> MaxIntegral;
	
	Pout = pout + iout + dout;

	pid->Error_2 = pid->Error_1;	// e(k-2)
	pid->Error_1 = pid->Error_0; 	// e(k-1)
	
	if(Pout > 1 || Pout < -1)
	{
		pid->Output += Pout; // 累加
	}// 防抖
	 // 输出限幅
    if(pid->Output > pid->MaxOutput)
	{
		pid->Output = pid->MaxOutput;
	}
		else if(pid->Output < -pid->MaxOutput)
	{
		pid->Output = -pid->MaxOutput;
	}
	
	return Pout;
}

float PID_Angle(PID *pid,float Actual)
{
	//记录实际值
	pid -> Actual = Actual;

	//目标值 - 实际值 得到误差值
	pid -> Error_0 = pid -> Target - pid -> Actual;

	//误差值累加得到误差积分
	pid -> Error_Sum += pid -> Error_0;

	//积分限幅
	pid -> Error_Sum = pid -> Error_Sum < pid -> MaxIntegral ? pid -> Error_Sum : pid -> MaxIntegral;
	pid -> Error_Sum = pid -> Error_Sum > -pid -> MaxIntegral ? pid -> Error_Sum : -pid -> MaxIntegral;
	
	//PID运算
	pid -> Actual = pid -> Kp * pid -> Error_0 + pid -> Ki * pid -> Error_Sum + pid -> Kd * (pid -> Error_0 - pid -> Error_Last);
	
	//保存上一次的值
	pid -> Error_Last = pid -> Error_0;
	
	//返回PID输出
	return pid -> Actual;
}
