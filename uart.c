#include "board.h"
#include "uart.h"
#include "motor.h"

#if EN_UART_0_RX
u16 UART_0_RX_STA=0;
char UART_0_RX_BUF[UART_RX_BUF_LEN];

void UART_0_send_char(char ch)
{
	while(DL_UART_isBusy(UART_0_INST));
	DL_UART_Main_transmitData(UART_0_INST,ch);
}

void UART_0_printf(char * str)
{
	while(*str != 0 && str != 0)
	{
		UART_0_send_char(*str++);
	}
}

//void UART_0_INST_IRQHandler()
//{
//	u8 RES;
//	switch(DL_UART_getPendingInterrupt(UART_0_INST))
//	{
//		case DL_UART_IIDX_RX:
//			RES = DL_UART_Main_receiveData(UART_0_INST);
//			if( !(UART_0_RX_STA & 0x8000) )//接收未完成
//			{
//				if( UART_0_RX_STA & 0x4000 )//接收到第一个帧尾
//				{
//					if( RES != 0x0A ) UART_0_RX_STA = 0;//若第二帧尾错误则重新接收
//					else UART_0_RX_STA |= 0x8000;
//				}
//				else
//				{
//					if( UART_0_RX_STA == 0 && RES == 0x55 )//接收帧头
//					{
//						UART_0_RX_BUF[UART_0_RX_STA & 0x3FFF] = RES;
//						UART_0_RX_STA++;
//					}
//					else
//					{
//						UART_0_RX_BUF[UART_0_RX_STA & 0x3FFF] = RES;
//						UART_0_RX_STA++;
//						if( RES == 0xff )
//						{
//							UART_0_RX_BUF[(UART_0_RX_STA & 0x3FFF)-1] = '\0';
//							UART_0_RX_STA |= 0x8000;
//						}
//					}
//					if( (UART_0_RX_STA & 0x3FFF) > UART_RX_BUF_LEN )UART_0_RX_STA = 0;
//				}
//			}
//			if( UART_0_RX_STA & 0x8000 )
//			{
//				//yaw = atoi(UART_0_RX_BUF+1);
//				UART_0_RX_STA = 0;
//			}
//			break;
//		default:
//			break;
//	}
//}

extern double angular_z;
extern uint16_t  BUFFER_SIZE;
extern uint16_t RxBuffer1[11];
int Res;
static uint8_t RxCounter1 = 0;
static uint8_t RxState = 0;



#define FRAME_HEADER_1 0x55
#define FRAME_HEADER_2 0x53


void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) == DL_UART_IIDX_RX)
    {
        Res = DL_UART_Main_receiveData(UART_0_INST);

        switch (RxState)
        {
        case 0:
            if (Res == FRAME_HEADER_1)
            {
                RxState = 1;
                RxBuffer1[RxCounter1++] = Res;
            }
            break;

        case 1:
            if (Res == FRAME_HEADER_2)
            {
                RxState = 2;
                RxBuffer1[RxCounter1++] = Res;
            }
            break;

        case 2:
            RxBuffer1[RxCounter1++] = Res;
            if (RxCounter1 >= 11 || (Res << 8) == ((RxBuffer1[0] + RxBuffer1[1] + RxBuffer1[2] + RxBuffer1[3] + RxBuffer1[4] + RxBuffer1[5] + RxBuffer1[6] + RxBuffer1[7] + RxBuffer1[8] + RxBuffer1[9]) << 8))
            {
                RxCounter1 = 0;
                RxState = 0;
								Get_Z();
            }
            break;

        default:
            RxState = 0;
            RxCounter1 = 0;
            break;
        }

        NVIC_ClearPendingIRQ(UART0_INT_IRQn);
    }
}

void Get_Z()
{
	angular_z = ((int)((RxBuffer1[7]<<8)|RxBuffer1[6]))/32768.0*180.0;
	if(angular_z >=180)
	{
		angular_z = -(360 - angular_z);
	}
}
//	

#endif

#if EN_UART_1_RX
u16 UART_1_RX_STA=0;
char UART_1_RX_BUF[UART_RX_BUF_LEN];

void UART_1_send_char(char ch)
{
	while(DL_UART_isBusy(UART_1_INST));
	DL_UART_Main_transmitData(UART_1_INST,ch);
}

void UART_1_printf(char * str)
{
	while(*str != 0 && str != 0)
	{
		UART_1_send_char(*str++);
	}
}

void UART_1_INST_IRQHandler()
{
	u8 RES;
	switch(DL_UART_getPendingInterrupt(UART_1_INST))
	{
		case DL_UART_IIDX_RX:
			RES = DL_UART_Main_receiveData(UART_1_INST);
			UART_0_send_char(RES);
		if(RES == 0x01)back();
		else if(RES == 0x02)stop();
		else if(RES == 0x03)stright();
		else if(RES == 0x04)left();
		else if(RES == 0x05)right();
			break;
		default:
			break;
	}
}

void uart1_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        UART_1_send_char(*str++);
    }
}

void HMISendb(u8 k)//·￠?íèy??0xff
{
	u8 i;
	for(i=0;i<3;i++)
	{
		if(k!=0)
		{
			 DL_UART_Main_transmitData(UART_1_INST, 0xff);
			while( DL_UART_isBusy(UART_1_INST) == true );
		}
		else return;
	}
}


void HMISendstart(void)
{
	delay_ms(20);
	HMISendb(0xff);
	delay_ms(20);	
}

//void UART_1_INST_IRQHandler()
//{
//	u8 RES;
//	switch(DL_UART_getPendingInterrupt(UART_1_INST))
//	{
//		case DL_UART_IIDX_RX:
//			RES = DL_UART_Main_receiveData(UART_1_INST);
//			if( !(UART_1_RX_STA & 0x8000) )//接收未完成
//			{
//				if( UART_1_RX_STA & 0x4000 )//接收到第一个帧尾
//				{
//					if( RES != 0x0A ) UART_1_RX_STA = 0;//若第二帧尾错误则重新接收
//					else UART_1_RX_STA |= 0x8000;
//				}
//				else
//				{
//					if( UART_1_RX_STA == 0 && RES == 0x55 )//接收帧头
//					{
//						UART_1_RX_BUF[UART_1_RX_STA & 0x3FFF] = RES;
//						UART_1_RX_STA++;
//					}
//					else
//					{
//						UART_1_RX_BUF[UART_1_RX_STA & 0x3FFF] = RES;
//						UART_1_RX_STA++;
//						if( RES == 0x0D )
//						{
//							UART_1_RX_BUF[(UART_1_RX_STA & 0x3FFF)-1] = '\0';
//							UART_1_RX_STA |= 0x4000;
//						}
//					}
//					if( (UART_1_RX_STA & 0x3FFF) > UART_RX_BUF_LEN )UART_1_RX_STA = 0;
//				}
//			}
//			if( UART_1_RX_STA & 0x8000 )
//			{
//				UART_0_printf(UART_1_RX_BUF+1);
//				UART_1_RX_STA = 0;
//			}
//			break;
//		default:
//			break;
//	}
//}


#endif


#if EN_UART_2_RX
u16  UART_2_RX_STA=0;
char UART_2_RX_BUF[UART_RX_BUF_LEN];

void UART_2_send_char(char ch)
{
	while(DL_UART_isBusy(UART_2_INST));
	DL_UART_Main_transmitData(UART_2_INST,ch);
}

void UART_2_printf(char * str)
{
	while(*str != 0 && str != 0)
	{
		UART_2_send_char(*str++);
	}
}

void UART_2_INST_IRQHandler()
{
	u8 RES;
	switch(DL_UART_getPendingInterrupt(UART_2_INST))
	{
		case DL_UART_IIDX_RX:
			RES = DL_UART_Main_receiveData(UART_2_INST);
			if( !(UART_2_RX_STA & 0x8000) )//接收未完成
			{
				if( UART_2_RX_STA & 0x4000 )//接收到第一个帧尾
				{
					if( RES != 0x0A ) UART_2_RX_STA = 0;//若第二帧尾错误则重新接收
					else UART_2_RX_STA |= 0x8000;
				}
				else
				{
					if( UART_2_RX_STA == 0 && RES == 0x55 )//接收帧头
					{
						UART_2_RX_BUF[UART_2_RX_STA & 0x3FFF] = RES;
						UART_2_RX_STA++;
					}
					else
					{
						UART_2_RX_BUF[UART_2_RX_STA & 0x3FFF] = RES;
						UART_2_RX_STA++;
						if( RES == 0x0D )
						{
							UART_2_RX_BUF[(UART_2_RX_STA & 0x3FFF)-1] = '\0';
							UART_2_RX_STA |= 0x4000;
						}
					}
					if( (UART_2_RX_STA & 0x3FFF) > UART_RX_BUF_LEN )UART_2_RX_STA = 0;
				}
			}
			if( UART_2_RX_STA & 0x8000 )
			{
				UART_2_printf(UART_2_RX_BUF+1);
				UART_2_RX_STA = 0;
			}
			break;
		default:
			break;
	}
}
#endif
