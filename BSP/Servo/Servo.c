#include "Servo.h"
#include "main.h"
uint16_t Angle;

//   0.5ms--------------0度；
//   1.0ms------------45度；
//   1.5ms------------90度； 1500
//   2.0ms-----------135度；
//   2.5ms-----------180度；
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
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600抱紧 ，2000初始化�    50000吃死     //3750锁紧  1800初始化
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
			/**************投掷前先松开抱球装置*******************/
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600抱紧 ，2000初始化�    50000吃死     //3750锁紧  1800初始化
			  TIM_SetCompare2(TIM9,RIGHT_TZ2_INIT);
			/**************投掷前先松开抱球装置*******************/
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
			/**************投掷前先松开抱球装置*******************/
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600抱紧 ，2000初始化�    50000吃死     //3750锁紧  1800初始化
			  TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
			/**************投掷前先松开抱球装置*******************/
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
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //正在交接   //延时的时候，RELAY_STATE == RELAY_SUCCEEDED，需要添加RELAY_STATE == RELAY_SUCCEEDED
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
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600抱紧 ，2000初始化�    50000吃死     //3750锁紧  1800初始化
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
//				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600抱紧 ，2000初始化�    50000吃死     //3750锁紧  1800初始化
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
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //正在交接   //延时的时候，RELAY_STATE == RELAY_SUCCEEDED，需要添加RELAY_STATE == RELAY_SUCCEEDED
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
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //正在交接   //延时的时候，RELAY_STATE == RELAY_SUCCEEDED，需要添加RELAY_STATE == RELAY_SUCCEEDED
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_LOCKED);
				TIM_SetCompare2(TIM9,RIGHT_SERVOR_LOCKED);
			}
		}
		break;

		default:
		{

			#if DEBUG_SERVO
			TIM_SetCompare1(TIM9,ggg);    //4600抱紧 ，2000初始化�    50000吃死     //3750锁紧  1800初始化
			TIM_SetCompare2(TIM9,gggggg);//1750抱紧，2500初始化，3600完全垂直张开 // 850锁紧   3000初始化
			#else
			TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600抱紧 ，2000初始化�    50000吃死     //3750锁紧  1800初始化
			TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);//1750抱紧，2500初始化，3600完全垂直张开 // 850锁紧   3000初始化
			#endif
			
//			TIM_SetCompare1(TIM1,ggg);
//			TIM_SetCompare1(TIM12,gggggg);
			
		}
	}

}



//PA8  TIM1-CH1
//PWM输出初始化  
//arr：自动重装值  
//psc：时钟预分频数
void TIM1_CH1_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //GPIOB3复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM2 Channe2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC2

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM14在CCR2上的预装载寄存器
	
 
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIM2								  
}  


void TIM3_CH3_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3); //GPIOB3复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //初始化PB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM2 Channe2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 4OC2

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM34在CCR2上的预装载寄存器
	
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM2								  
}  





void TIM9_CH2_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9); //GPIOB3复用为定时器14
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //GPIOB3复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM2 Channe2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM9 4OC2

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM94在CCR2上的预装载寄存器
	
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM9 4OC2

	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM94在CCR2上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM9, ENABLE);  //使能TIM2								  
}  


void TIM12_CH1_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12); //GPIOB3复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM2 Channe2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM12 4OC2

	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);  //使能TIM124在CCR2上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM12,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM12, ENABLE);  //使能TIM2								  
}  



