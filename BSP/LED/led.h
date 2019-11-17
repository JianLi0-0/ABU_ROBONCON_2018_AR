#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED�˿ڶ���
#define LED0 PFout(9)	// SYS_LED
#define LED1 PFout(10)	// SYS_LED

/*F3F5 LED1,LED2*/
/*E2E4 LED3,LED4*/
#define ERROR_LED1 PAout(15)	
#define ERROR_LED2 PBout(15)	 
#define ERROR_LED3 PGout(10)	
#define ERROR_LED4 PDout(7)	 

#define ERROR_LED2_ON()            GPIO_ResetBits(GPIOB, GPIO_Pin_15)
#define ERROR_LED2_OFF()           GPIO_SetBits(GPIOB, GPIO_Pin_15)
#define ERROR_LED2_TOGGLE()        GPIO_ToggleBits(GPIOB, GPIO_Pin_15)

#define ERROR_LED1_ON()            GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define ERROR_LED1_OFF()           GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define ERROR_LED1_TOGGLE()        GPIO_ToggleBits(GPIOA, GPIO_Pin_15)

#define ERROR_LED3_ON()            GPIO_ResetBits(GPIOG, GPIO_Pin_10)
#define ERROR_LED3_OFF()           GPIO_SetBits(GPIOG, GPIO_Pin_10)
#define ERROR_LED3_TOGGLE()        GPIO_ToggleBits(GPIOG, GPIO_Pin_10)

#define ERROR_LED4_ON()            GPIO_ResetBits(GPIOD, GPIO_Pin_7)
#define ERROR_LED4_OFF()           GPIO_SetBits(GPIOD, GPIO_Pin_7)
#define ERROR_LED4_TOGGLE()        GPIO_ToggleBits(GPIOD, GPIO_Pin_7)

void LED_Init(void);//��ʼ��		 				    
#endif
