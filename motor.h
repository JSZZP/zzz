#ifndef __motor_H
#define __motor_H

#include "ti_msp_dl_config.h"
#define u8 uint8_t

void Work_Mode(u8 Lmode, u8 Rmode);
void stright();
void left();
void right();
void stop();
void back();
void Motor(int * Lmotor, int * Rmotor);
void Read_Data();
void Feed_Back();

#endif