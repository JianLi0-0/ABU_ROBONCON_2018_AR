#include "sys.h"
#include "usart.h"	
#include "stdlib.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F4探索者开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
//int fputc(int ch, FILE *f)
//{ 	
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART1->DR = (u8) ch;      
//	return ch;
//}

int fputc(int ch, FILE *f)
{ 	

	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}
/*******************************上位机调试***********************************/
union
{
	char A[8];
	char B[8];
	char C[8];
	char D[8];
	char E[8];
	char F[8];
	char G[8];
	char H[8];
	char I[8];
	char J[8];
}TP;


float Throw[10];
	
//char openmv_yaw_data[8];
//char openmv_x_data[8];
//char openmv_z_data[8];
//float openmv_x=0.0;
//float openmv_yaw=0.0;
//float openmv_z=0.0;

int count_cam=0;
extern uint16_t tag;

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	static int i;
	
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
//		tag = 2000;
		switch(count_cam)
		{
			
			case 0:
			{
						 if(Res==0x01) count_cam = 1;
				else if(Res==0x02) count_cam = 2;
				else if(Res==0x03) count_cam = 3;
				else if(Res==0x04) count_cam = 4;
				else if(Res==0x05) count_cam = 5;
				else if(Res==0x06) count_cam = 6;
				else if(Res==0x07) count_cam = 7;
				else if(Res==0x08) count_cam = 8;
				else if(Res==0x09) count_cam = 9;
				else if(Res==0x0a) count_cam = 10;
			}break;
			case 1:
			{
				TP.A[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[0] = atof(TP.A);
					}
			}break;
			case 2:
			{
				TP.B[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[1] = atof(TP.B)*10.0;
					}
			}break;
			case 3:
			{
				TP.C[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[2] = atof(TP.C)*10.0;
					}
			}break;
			case 4:
			{
				TP.D[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[3] = atof(TP.D)*10.0;
					}
			}break;
			case 5:
			{
				TP.E[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[4] = atof(TP.E)*10.0;
					}
			}break;
			case 6:
			{
				TP.F[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[5] = atof(TP.F)*10.0;
					}
			}break;
			case 7:
			{
				TP.G[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[6] = atof(TP.G)*10.0;
					}
			}break;
			case 8:
			{
				TP.H[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[7] = atof(TP.H)*10.0;
					}
			}break;
			case 9:
			{
				TP.I[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[8] = atof(TP.I)*10.0;
					}
			}break;
			case 10:
			{
				TP.J[i] = Res;
				i++;
				if(i>=6) 
					{
						count_cam = 0,i = 0;
				    Throw[9] = atof(TP.J)*10.0;
					}
			}break;

	  }
  } 
}
/*******************************上位机调试***********************************/



#endif	


