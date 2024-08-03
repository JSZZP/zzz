#ifndef __ODOMETER_H
#define __ODOMETER_H




int int_abs(int in);
float float_abs(float in);
int Laps_limit(float Now_Laps,float Target_Laps);
void Odom_Get(float LSpeed,float RSpeed);
void Odom_Turn(float Act_Ang,float Tar_Ang);
void Task3(float Act_Ang,int TAR_ANG,int dis);
void Task1(float Act_Ang,float Act_Omega);
void Task2(float Act_Ang);
#endif
