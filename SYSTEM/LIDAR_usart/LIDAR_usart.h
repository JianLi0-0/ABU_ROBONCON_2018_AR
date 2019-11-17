#ifndef __LIDAR_USART_H
#define __LIDAR_USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
	
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收


typedef struct lidar
{
	float pos_x;
	float pos_y;
	float zangle;
	float xangle;
  float yangle;
  float w_z;
}lidar;

extern lidar LIDAR_POS;

void Action_uart_init(u32 bound);
void send_pos(float pos_x, float pos_y, float zangle);
#endif


