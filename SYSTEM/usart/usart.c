#include "sys.h"
#include "usart.h"	
#include "stdlib.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4̽���߿�����
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/6/10
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
//int fputc(int ch, FILE *f)
//{ 	
//	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
//	USART1->DR = (u8) ch;      
//	return ch;
//}

int fputc(int ch, FILE *f)
{ 	

	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}
/*******************************��λ������***********************************/
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

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	static int i;
	
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
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
/*******************************��λ������***********************************/



#endif	


