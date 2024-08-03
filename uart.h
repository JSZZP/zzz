#ifndef __uart_H
#define __uart_H

#include "ti_msp_dl_config.h"
#define u16 uint16_t
#define u8 uint8_t
#define UART_RX_BUF_LEN 64

#define EN_UART_0_RX 1
#define EN_UART_1_RX 1
#define EN_UART_2_RX 0

void UART_0_send_char(char ch);
void UART_0_printf(char * str);
void UART_1_send_char(char ch);
void UART_1_printf(char * str);
void uart1_send_string(char* str);
void HMISendb(u8 k);
void HMISendstart(void);
void UART_2_send_char(char ch);
void UART_2_printf(char * str);
void Get_Z();
#endif