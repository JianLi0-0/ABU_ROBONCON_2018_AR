#include "main.h"


u32 Colour_num=0;								  // ������ɫ�������������
u8  Time7_flag=0;                 // ��ʱ���жϱ�־

////////////////////////////////////////////////////////////////////////////////////

/*==============================����Ϊ��ʱ��3���򲿷�=================================*/
//ͨ�ö�ʱ��7�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��7!
void TIM7_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///ʹ��TIM7ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//��ʼ��TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //����ʱ��7�����ж�
	TIM_Cmd(TIM7,ENABLE); //ʹ�ܶ�ʱ��7
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //��ʱ��7�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/*��ʱ��7�жϷ�����*/
//��ʱ������
void TIM7_IRQHandler(void)
{ 
 
  if(TIM7->SR&0X0001)//����ж�
   {
	 Time7_flag=1;
	 EXTI->IMR|=0<<0;//�ر��ⲿ�ж�0
	 TIM7->CR1|=0X00;	//�رն�ʱ��7
   }
   TIM7->SR&=0<<0;//����жϱ�־λ 

}
/////////////////////////////////////////////////////////////////////////////////////


/*=============================����Ϊ�ⲿ�ж�0���򲿷�=================================*/
//�ⲿ�жϳ�ʼ������
//��ʼ��PB0Ϊ�ж�����.������ɫ��������out�������
void TCS230_EXTIX_Init(void)
{

	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
   

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);//PB0 ���ӵ��ж���0

  /* ����EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�����ش��� 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
  EXTI_Init(&EXTI_InitStructure);//����

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//�ⲿ�ж�0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
}

/*�ⲿ�ж�0�������*/
void EXTI0_IRQHandler(void)
{
	Colour_num++;
  EXTI->PR = 1<<0; //���line0�ϵ��жϱ�־λ
	EXTI->IMR|=0<<0;//�ر��ⲿ�ж�0
	TIM7->CR1|=0X00;	//�رն�ʱ��7
}

/**********************************************************
 �������ƣ�TCS230_Init
 �������ܣ�TCS230��ʼ��
 ��ڲ�������
 ���ز�������
 ��    ע����ʼ���ⲿ�ж�0����ʱ���ж�3�����I/O������
**********************************************************/ 
void TCS230_Init(void)
{
 	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��
		 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // TCS230�������out��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB0
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; // TCS230�������S2,S3��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE0,GPIOE1
	
	
	TIM7_Int_Init(500,8400);//50ms
	TCS230_EXTIX_Init();
	
}	
	
/***********************************************
 �������ƣ�Colour_Init
 �������ܣ�TCS230�ڲ���ɫ�˲�������
 ��ڲ�����Colour
 ���ز�������
 ��    ע��Colour ѡ����ɫ��Red1/Blue1/Green1��
************************************************/ 	
void Colour_Init(u8 Colour)
{
	switch(Colour)
	 {
	 	case Red1  : TCS32_S2=0;TCS32_S3=0;break;
		case Blue1 : TCS32_S2=0;TCS32_S3=1;break;
		case Green1: TCS32_S2=1;TCS32_S3=1;break;
		case White1: TCS32_S2=1;TCS32_S3=0;break;
		default    : break;
	 }
}

/**********************************************************
 �������ƣ�TCS230_Start
 �������ܣ���ʼ������������ʱ��3���ⲿ�ж�0�������������
 ��ڲ�������
 ���ز�������
**********************************************************/ 
void TCS230_Start(void)
{		
    TIM7->CR1|=0X01;	//ʹ�ܶ�ʱ��7
		Colour_num=0;//	��������
		Time7_flag=0; // ��ʱ����־����
	  EXTI->IMR|=1<<0;//�����ⲿ�ж�0	????????????????????
}



/******************************************
 �������ƣ�TCS230_Read_Colour
 �������ܣ�������ɫ�˲�����������ӦƵ�ʲ���
 ��ڲ�����Colour
 ���ز�����Colour num
 ��    ע��Colour     ����ɫ�˲������ò���
           Colour num ���������
******************************************/ 
u32 TCS230_Read_Colour(u8 Colour)
{
	Colour_Init(Colour);//����TCS230����ɫ�˲���
	delay_ms(35);
	TCS230_Start();//��ʼ������������ʱ��3���ⲿ�ж�0
	while(Time7_flag==0);//�ȴ��������
	return Colour_num;	// ���ؼ�������
}



/******************************************
 �������ƣ�TCS230_Distinguish
 �������ܣ�������ɫ�б�
 ��ڲ�����    
 ���ز�����1/2/3
	         flag_color=r,i=1;red;
	         flag_color=g,i=2;green;
	         flag_color=b;i=3;blue;
 ��    ע��0:Red1
           1:Blue1
		       2:Green1
		        

******************************************/ 
int TCS230_Distinguish(void)
{
	int i=1;
	int r=0 ,b=0 ,g=0,flag_color=0 ;	
  r=TCS230_Read_Colour(Red1);
	g=TCS230_Read_Colour(Green1);
	b=TCS230_Read_Colour(Blue1);
	flag_color=r;
	if (flag_color<g)   {flag_color=g;i=2;}
	if (flag_color<b)   {flag_color=b;i=3;}	

	
	
	printf("r:%d \r\n",r);
		printf("g:%d \r\n",g);
		printf("b:%d  \r\n",b);
//	
//	{switch ( i )
//	{
//		case 1:printf("red");break;
//		case 2:printf("green");break;
//		case 3:printf("blue");break;
//		break;
//	}
//	 return 0;}	

	return i;
}














