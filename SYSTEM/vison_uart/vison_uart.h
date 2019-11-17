#ifndef __VISION_UART_H
#define __VISION_UART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
	
#define EN_UART5_RX 			1		//使能（1）/禁止（0）串口1接收



void Vision_uart5_init(u32 bound);

#endif


