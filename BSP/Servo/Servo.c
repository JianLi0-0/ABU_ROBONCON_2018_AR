#include "Servo.h"
#include "main.h"
uint16_t Angle;

//   0.5ms--------------0�ȣ�
//   1.0ms------------45�ȣ�
//   1.5ms------------90�ȣ� 1500
//   2.0ms-----------135�ȣ�
//   2.5ms-----------180�ȣ�
//0.5ms-2.5ms

uint32_t ggg = 500,gggggg = 2500;


void Servo_Task_init(void)
{
//	TIM1_CH1_PWM_Init(20000-1,83);
//	TIM3_CH3_PWM_Init(20000-1,83);
	TIM9_CH2_PWM_Init(20000-1,83);
//	TIM12_CH1_PWM_Init(20000-1,83);
	TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);
	TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
	
//	TIM_SetCompare1(TIM1,LEFT_SERVOR_INIT);
//  TIM_SetCompare3(TIM3,LEFT_CLAMP_INIT);
//	TIM_SetCompare1(TIM12,RIGHT_CLAMP_INIT);
}
void Servo_Task(void) //20ms
{
	switch( NOW_STATE )
	{
		
		case RELAY1_TO_TZ2 :
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP  && localize  succeeded
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
			  TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
			}
			if(TRACKING_STATE != TRACKING_ARRIVED)
			{
				TIM_SetCompare1(TIM9,LEFT_TZ2_LOCKED);
				TIM_SetCompare2(TIM9,RIGHT_SERVOR_LOCKED);
			}
		}
		break;
		
		
		case RELAY1_TO_TZ1 : 
		{
		
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP  && localize  succeeded
			{
			/**************Ͷ��ǰ���ɿ�����װ��*******************/
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
			  TIM_SetCompare2(TIM9,RIGHT_TZ2_INIT);
			/**************Ͷ��ǰ���ɿ�����װ��*******************/
			}
			
			if(TRACKING_STATE != TRACKING_ARRIVED)
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_LOCKED);
				TIM_SetCompare2(TIM9,RIGHT_SERVOR_LOCKED);
			}
		}
		break;
		
		
		case RELAY2_TO_TZ2 :
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP  && localize  succeeded
			{
			/**************Ͷ��ǰ���ɿ�����װ��*******************/
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
			  TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
			/**************Ͷ��ǰ���ɿ�����װ��*******************/
			}
			
			if(TRACKING_STATE != TRACKING_ARRIVED)
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_LOCKED);
				TIM_SetCompare2(TIM9,RIGHT_SERVOR_LOCKED);
			}
		}
		break;
		
		
		
		case RELAY_STATE4 : 
		{
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //���ڽ���   //��ʱ��ʱ��RELAY_STATE == RELAY_SUCCEEDED����Ҫ���RELAY_STATE == RELAY_SUCCEEDED
			{
//				TIM_SetCompare1(TIM9,LEFT_TZ3_LOCKED);
//				TIM_SetCompare2(TIM9,RIGHT_TZ3_LOCKED);
			}
		}
		break;
		case RELAY2_TO_TZ3 : 
		{
			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP  && localize  succeeded
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
			  TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
			}
			else if(TRACKING_STATE != TRACKING_ARRIVED)
			{
//				if(goal_errors.ran_distance/goal_errors.distance_between_2pts<0.1
//				if(goal_errors.ran_distance>0.15*goal_errors.distance_between_2pts
//					 && goal_errors.ran_distance<=0.2*goal_errors.distance_between_2pts)
//				{
//					TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);
//					TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
//				}
//				else 
				if(goal_errors.ran_distance>0.3*goal_errors.distance_between_2pts)
				{
					TIM_SetCompare1(TIM9,LEFT_TZ3_LOCKED);
					TIM_SetCompare2(TIM9,RIGHT_TZ3_LOCKED);
				}
			}
//			if(TRACKING_STATE == TRACKING_ARRIVED) //AA TP  && localize  succeeded
//			{
//				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
//			  TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
//			}
//			if(TRACKING_STATE != TRACKING_ARRIVED)
//			{
////				if(goal_errors.ran_distance/goal_errors.distance_between_2pts<0.1
//				TIM_SetCompare1(TIM9,LEFT_TZ3_LOCKED);
//				TIM_SetCompare2(TIM9,RIGHT_TZ3_LOCKED);
//			}
		}
		break;
		
		
		
		//case RELAY_STATE1 : 
		case RELAY_STATE2 : 
		{
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //���ڽ���   //��ʱ��ʱ��RELAY_STATE == RELAY_SUCCEEDED����Ҫ���RELAY_STATE == RELAY_SUCCEEDED
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_LOCKED);
				TIM_SetCompare2(TIM9,RIGHT_TZ2_INIT);
			}
			else
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);
				TIM_SetCompare2(TIM9,RIGHT_TZ2_INIT);
			}
		}
		break;
			
		case RELAY_STATE3 : 
		{
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //���ڽ���   //��ʱ��ʱ��RELAY_STATE == RELAY_SUCCEEDED����Ҫ���RELAY_STATE == RELAY_SUCCEEDED
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_LOCKED);
				TIM_SetCompare2(TIM9,RIGHT_SERVOR_LOCKED);
			}
		}
		break;

		default:
		{

			#if DEBUG_SERVO
			TIM_SetCompare1(TIM9,ggg);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
			TIM_SetCompare2(TIM9,gggggg);//1750������2500��ʼ����3600��ȫ��ֱ�ſ� // 850����   3000��ʼ��
			#else
			TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600���� ��2000��ʼ���    50000����     //3750����  1800��ʼ��
			TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);//1750������2500��ʼ����3600��ȫ��ֱ�ſ� // 850����   3000��ʼ��
			#endif
			
//			TIM_SetCompare1(TIM1,ggg);
//			TIM_SetCompare1(TIM12,gggggg);
			
		}
	}

}



//PA8  TIM1-CH1
//PWM�����ʼ��  
//arr���Զ���װֵ  
//psc��ʱ��Ԥ��Ƶ��
void TIM1_CH1_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //GPIOB3����Ϊ��ʱ��14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��4
	
	//��ʼ��TIM2 Channe2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC2

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR2�ϵ�Ԥװ�ؼĴ���
	
 
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM2								  
}  


void TIM3_CH3_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3); //GPIOB3����Ϊ��ʱ��14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //��ʼ��PB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��4
	
	//��ʼ��TIM2 Channe2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 4OC2

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM34��CCR2�ϵ�Ԥװ�ؼĴ���
	
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM2								  
}  





void TIM9_CH2_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM14ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9); //GPIOB3����Ϊ��ʱ��14
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //GPIOB3����Ϊ��ʱ��14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //��ʼ��PB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);//��ʼ����ʱ��4
	
	//��ʼ��TIM2 Channe2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM9 4OC2

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM94��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM9 4OC2

	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //ʹ��TIM94��CCR2�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM9, ENABLE);  //ʹ��TIM2								  
}  


void TIM12_CH1_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTFʱ��	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12); //GPIOB3����Ϊ��ʱ��14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //��ʼ��PB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);//��ʼ����ʱ��4
	
	//��ʼ��TIM2 Channe2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM12 4OC2

	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);  //ʹ��TIM124��CCR2�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM12,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM12, ENABLE);  //ʹ��TIM2								  
}  



