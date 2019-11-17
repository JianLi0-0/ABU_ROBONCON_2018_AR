#include "vison_uart.h"
#include "main.h"

#define LIDAR 0
#define OPENMV 1
char VISION_MES;
char VISION_MES_temp;
void UART5_IRQHandler(void)                	//����1�жϷ������
{
	static uint8_t ch;
	static union
	{
	 uint8_t data[24];
	 float ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t v_count=0;
  static uint8_t i=0;

	if(USART_GetITStatus(UART5, USART_IT_RXNE) == SET)   
	{
		USART_ClearITPendingBit( UART5,USART_IT_RXNE);	
		ch=USART_ReceiveData(UART5);
		#if LIDAR
		switch(count)
		 {
			 case 0:
				 if(ch==0x0d)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 1:
				 if(ch==0x0a)
				 {
					 i=0;
					 count++;
				 }
				 else if(ch==0x0d);
				 else
					 count=0;
				 break;
				 
			 case 2:
				 posture.data[i]=ch;
			   i++;
			   if(i>=24)
				 {
					 i=0;
					 count++;
				 }
				 break;
				 
			 case 3:
				 if(ch==0x0a)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 4:
				 if(ch==0x0d)
				 {
  				 LIDAR_POS.zangle=posture.ActVal[0];
	  		   LIDAR_POS.xangle=posture.ActVal[1];
		  	   LIDAR_POS.yangle=posture.ActVal[2];
			     LIDAR_POS.pos_x =posture.ActVal[3];
			     LIDAR_POS.pos_y =posture.ActVal[4];
			     LIDAR_POS.w_z   =posture.ActVal[5];
				 }
			   count=0;
				 break;
			 
			 default:
				 count=0;
			   break;		 
		 }
		 #endif
//		 LostCounterFeed(&missing_counter[MISSING_COUNTER_INDEX_LIDAR]);/*software watchdog*/
		 #if OPENMV
		 switch(v_count)
		 {
			 case 0:
				 if(ch==0x0a)
					 v_count++;
				 else
					 v_count=0;
				 break;
				 
			 case 1:
				 if(ch==0x0a)
				 {
					 v_count++;
				 }
				 else
					 v_count=0;
				 break;
				 
			 case 2:
				 VISION_MES_temp=ch;
			   v_count++;
				 break;
				 
			 case 3:
				 if(ch==0x0b)
					 v_count++;
				 else
					 v_count=0;
				 break;
				 
			 case 4:
				 if(ch==0x0b)
				 {
  				 VISION_MES=VISION_MES_temp;
				 }
			   v_count=0;
				 break;
			 
			 default:
				 v_count=0;
			   break;		 
		 }
		 #endif

	 }
}


void Vision_uart5_init(u32 bound)
{
	//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOAʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //GPIOA9��GPIOA10  //TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PA9��PA10
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOA9��GPIOA10  //RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��PA9��PA10
	
	
   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART5, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(UART5, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_UART5_RX	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
}
