#include "encoder.h"
 

	//������A---A�ࣺPA6��B�ࣺPA7
void encoder_Init_TIM3(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
  //GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PA.06,07 as encoder input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;    //pA6 A�� pA7 B ��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;    //pA6 A�� pA7 B ��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	 
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = TIM_period_num; 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
 
  TIM_Cmd(TIM3, ENABLE); 
}


	//������B---A�ࣺPB6��B�ࣺPB7
void encoder_Init_TIM4(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//*
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//*
 
  //GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PB.06,07 as encoder input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;    //pB6 A�� pB7 B ��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);//*

  GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);//****
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);//****
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = TIM_period_num; 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//*
  
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//*
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);//*
  
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//*
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//*
  
  TIM_Cmd(TIM4, ENABLE); //*
}

//	//������C---A�ࣺPC6��B�ࣺPC7
//void encoder_Init_TIM8(void)
//{
//  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
//  TIM_ICInitTypeDef TIM_ICInitStructure;  
//  GPIO_InitTypeDef GPIO_InitStructure;
////  NVIC_InitTypeDef NVIC_InitStructure;
//  
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//*
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//*
//  
//  //GPIO_StructInit(&GPIO_InitStructure);
//  /* Configure PC6.7 as encoder input */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;    ///pC6 A�� pC7 B ��
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);//*

//  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);//***
//  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);//***
//  
//  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//  
//  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling 
//  TIM_TimeBaseStructure.TIM_Period = TIM_period_num; 
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
//  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);//*
//  

//  //  TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//*
//  TIM_ICStructInit(&TIM_ICInitStructure);
//  TIM_ICInitStructure.TIM_ICFilter = 0;
//  TIM_ICInit(TIM8, &TIM_ICInitStructure);//*
//  TIM_ClearFlag(TIM8, TIM_FLAG_Update);//*
//  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);//*
//  
//  TIM_Cmd(TIM8, ENABLE); //*
//}
void encoder_Init_TIM8(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);			//TIM8???????			**-?
 
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  								//TIM8_CH1?????			**-?  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);										//TIM8_CH1?????			**-?								
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;    							//TIM8_CH2?????			**-?
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);										//TIM8_CH2?????			**-?

  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);			//TIM8_CH1?GPIO??			**-?^2
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);			//TIM8_CH2?GPIO??			**-?^2
  

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
  TIM_TimeBaseStructure.TIM_Period = TIM_period_num; 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  
  TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 0;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	TIM8->CNT = 0;
  TIM_Cmd(TIM8, ENABLE); 
}


/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {	   
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		 case 8:  Encoder_TIM= (short)TIM8 -> CNT;  TIM8 -> CNT=0;break;
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
/**************************************************************************
�������ܣ���ȡλ����Ϣ
��ڲ�������ʱ��
����  ֵ��λ��ֵ���ص����ڲ���ռĴ���cnt��ֵ
**************************************************************************/
int Read_Position(u8 TIMX)
{
int Encoder_TIM;
switch(TIMX)
{
case 3: Encoder_TIM= (short)TIM3 -> CNT; break;
case 4: Encoder_TIM= (short)TIM4 -> CNT; break;
case 8: Encoder_TIM= (short)TIM8 -> CNT; break;
default: Encoder_TIM=0;
}
return Encoder_TIM;
}
/**************************************************************************
�������ܣ�TIM3-4-5�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{

	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ  
}

void TIM4_IRQHandler(void)
{ 		    		  			    
if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //����ж�
	{

	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //����жϱ�־λ  
}

void TIM8_IRQHandler(void)
{ 		    		  			    
if(TIM_GetITStatus(TIM8,TIM_IT_Update)==SET) //����ж�
	{

	}
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update);  //����жϱ�־λ  
}
