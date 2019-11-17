#include "Servo.h"
#include "main.h"
uint16_t Angle;

//   0.5ms--------------0∂»£ª
//   1.0ms------------45∂»£ª
//   1.5ms------------90∂»£ª 1500
//   2.0ms-----------135∂»£ª
//   2.5ms-----------180∂»£ª
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
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600±ßΩÙ £¨2000≥ı ºªØ£    50000≥‘À¿     //3750À¯ΩÙ  1800≥ı ºªØ
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
			/**************Õ∂÷¿«∞œ»À…ø™±ß«Ú◊∞÷√*******************/
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600±ßΩÙ £¨2000≥ı ºªØ£    50000≥‘À¿     //3750À¯ΩÙ  1800≥ı ºªØ
			  TIM_SetCompare2(TIM9,RIGHT_TZ2_INIT);
			/**************Õ∂÷¿«∞œ»À…ø™±ß«Ú◊∞÷√*******************/
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
			/**************Õ∂÷¿«∞œ»À…ø™±ß«Ú◊∞÷√*******************/
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600±ßΩÙ £¨2000≥ı ºªØ£    50000≥‘À¿     //3750À¯ΩÙ  1800≥ı ºªØ
			  TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);
			/**************Õ∂÷¿«∞œ»À…ø™±ß«Ú◊∞÷√*******************/
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
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //’˝‘⁄ΩªΩ”   //—” ±µƒ ±∫Ú£¨RELAY_STATE == RELAY_SUCCEEDED£¨–Ë“™ÃÌº”RELAY_STATE == RELAY_SUCCEEDED
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
				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600±ßΩÙ £¨2000≥ı ºªØ£    50000≥‘À¿     //3750À¯ΩÙ  1800≥ı ºªØ
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
//				TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600±ßΩÙ £¨2000≥ı ºªØ£    50000≥‘À¿     //3750À¯ΩÙ  1800≥ı ºªØ
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
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //’˝‘⁄ΩªΩ”   //—” ±µƒ ±∫Ú£¨RELAY_STATE == RELAY_SUCCEEDED£¨–Ë“™ÃÌº”RELAY_STATE == RELAY_SUCCEEDED
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
			if(RELAY_STATE == RELAY_GOING || RELAY_STATE == RELAY_SUCCEEDED)  //’˝‘⁄ΩªΩ”   //—” ±µƒ ±∫Ú£¨RELAY_STATE == RELAY_SUCCEEDED£¨–Ë“™ÃÌº”RELAY_STATE == RELAY_SUCCEEDED
			{
				TIM_SetCompare1(TIM9,LEFT_SERVOR_LOCKED);
				TIM_SetCompare2(TIM9,RIGHT_SERVOR_LOCKED);
			}
		}
		break;

		default:
		{

			#if DEBUG_SERVO
			TIM_SetCompare1(TIM9,ggg);    //4600±ßΩÙ £¨2000≥ı ºªØ£    50000≥‘À¿     //3750À¯ΩÙ  1800≥ı ºªØ
			TIM_SetCompare2(TIM9,gggggg);//1750±ßΩÙ£¨2500≥ı ºªØ£¨3600ÕÍ»´¥π÷±’≈ø™ // 850À¯ΩÙ   3000≥ı ºªØ
			#else
			TIM_SetCompare1(TIM9,LEFT_SERVOR_INIT);    //4600±ßΩÙ £¨2000≥ı ºªØ£    50000≥‘À¿     //3750À¯ΩÙ  1800≥ı ºªØ
			TIM_SetCompare2(TIM9,RIGHT_SERVOR_INIT);//1750±ßΩÙ£¨2500≥ı ºªØ£¨3600ÕÍ»´¥π÷±’≈ø™ // 850À¯ΩÙ   3000≥ı ºªØ
			#endif
			
//			TIM_SetCompare1(TIM1,ggg);
//			TIM_SetCompare1(TIM12,gggggg);
			
		}
	}

}



//PA8  TIM1-CH1
//PWM ‰≥ˆ≥ı ºªØ  
//arr£∫◊‘∂Ø÷ÿ◊∞÷µ  
//psc£∫ ±÷”‘§∑÷∆µ ˝
void TIM1_CH1_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM14 ±÷” πƒ‹    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	// πƒ‹PORTF ±÷”	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //GPIOB3∏¥”√Œ™∂® ±∆˜14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //∏¥”√π¶ƒ‹
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//ÀŸ∂»100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Õ∆ÕÏ∏¥”√ ‰≥ˆ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //…œ¿≠
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //≥ı ºªØPB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //∂® ±∆˜∑÷∆µ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //œÚ…œº∆ ˝ƒ£ Ω
	TIM_TimeBaseStructure.TIM_Period=arr;   //◊‘∂Ø÷ÿ◊∞‘ÿ÷µ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//≥ı ºªØ∂® ±∆˜4
	
	//≥ı ºªØTIM2 Channe2 PWMƒ£ Ω	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //—°‘Ò∂® ±∆˜ƒ£ Ω:TIM¬ˆ≥ÂøÌ∂»µ˜÷∆ƒ£ Ω1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //±»Ωœ ‰≥ˆ πƒ‹
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // ‰≥ˆº´–‘:TIM ‰≥ˆ±»Ωœº´–‘µÕ
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //∏˘æ›T÷∏∂®µƒ≤Œ ˝≥ı ºªØÕ‚…ËTIM1 4OC2

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  // πƒ‹TIM14‘⁄CCR2…œµƒ‘§◊∞‘ÿºƒ¥Ê∆˜
	
 
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE πƒ‹ 
	
	TIM_Cmd(TIM1, ENABLE);  // πƒ‹TIM2								  
}  


void TIM3_CH3_PWM_Init(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM14 ±÷” πƒ‹    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	// πƒ‹PORTF ±÷”	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3); //GPIOB3∏¥”√Œ™∂® ±∆˜14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //∏¥”√π¶ƒ‹
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//ÀŸ∂»100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Õ∆ÕÏ∏¥”√ ‰≥ˆ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //…œ¿≠
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //≥ı ºªØPB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //∂® ±∆˜∑÷∆µ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //œÚ…œº∆ ˝ƒ£ Ω
	TIM_TimeBaseStructure.TIM_Period=arr;   //◊‘∂Ø÷ÿ◊∞‘ÿ÷µ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//≥ı ºªØ∂® ±∆˜4
	
	//≥ı ºªØTIM2 Channe2 PWMƒ£ Ω	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //—°‘Ò∂® ±∆˜ƒ£ Ω:TIM¬ˆ≥ÂøÌ∂»µ˜÷∆ƒ£ Ω1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //±»Ωœ ‰≥ˆ πƒ‹
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // ‰≥ˆº´–‘:TIM ‰≥ˆ±»Ωœº´–‘µÕ
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //∏˘æ›T÷∏∂®µƒ≤Œ ˝≥ı ºªØÕ‚…ËTIM3 4OC2

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  // πƒ‹TIM34‘⁄CCR2…œµƒ‘§◊∞‘ÿºƒ¥Ê∆˜
	
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE πƒ‹ 
	
	TIM_Cmd(TIM3, ENABLE);  // πƒ‹TIM2								  
}  





void TIM9_CH2_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM14 ±÷” πƒ‹    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	// πƒ‹PORTF ±÷”	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9); //GPIOB3∏¥”√Œ™∂® ±∆˜14
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //GPIOB3∏¥”√Œ™∂® ±∆˜14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //∏¥”√π¶ƒ‹
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//ÀŸ∂»100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Õ∆ÕÏ∏¥”√ ‰≥ˆ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //…œ¿≠
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //≥ı ºªØPB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //∂® ±∆˜∑÷∆µ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //œÚ…œº∆ ˝ƒ£ Ω
	TIM_TimeBaseStructure.TIM_Period=arr;   //◊‘∂Ø÷ÿ◊∞‘ÿ÷µ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);//≥ı ºªØ∂® ±∆˜4
	
	//≥ı ºªØTIM2 Channe2 PWMƒ£ Ω	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //—°‘Ò∂® ±∆˜ƒ£ Ω:TIM¬ˆ≥ÂøÌ∂»µ˜÷∆ƒ£ Ω1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //±»Ωœ ‰≥ˆ πƒ‹
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // ‰≥ˆº´–‘:TIM ‰≥ˆ±»Ωœº´–‘µÕ
	
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //∏˘æ›T÷∏∂®µƒ≤Œ ˝≥ı ºªØÕ‚…ËTIM9 4OC2

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  // πƒ‹TIM94‘⁄CCR2…œµƒ‘§◊∞‘ÿºƒ¥Ê∆˜
	
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);  //∏˘æ›T÷∏∂®µƒ≤Œ ˝≥ı ºªØÕ‚…ËTIM9 4OC2

	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  // πƒ‹TIM94‘⁄CCR2…œµƒ‘§◊∞‘ÿºƒ¥Ê∆˜
 
  TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE πƒ‹ 
	
	TIM_Cmd(TIM9, ENABLE);  // πƒ‹TIM2								  
}  


void TIM12_CH1_PWM_Init(u32 arr,u32 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM12 ±÷” πƒ‹    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	// πƒ‹PORTF ±÷”	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12); //GPIOB3∏¥”√Œ™∂® ±∆˜14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //∏¥”√π¶ƒ‹
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//ÀŸ∂»100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Õ∆ÕÏ∏¥”√ ‰≥ˆ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //…œ¿≠
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //≥ı ºªØPB3
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //∂® ±∆˜∑÷∆µ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //œÚ…œº∆ ˝ƒ£ Ω
	TIM_TimeBaseStructure.TIM_Period=arr;   //◊‘∂Ø÷ÿ◊∞‘ÿ÷µ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);//≥ı ºªØ∂® ±∆˜4
	
	//≥ı ºªØTIM2 Channe2 PWMƒ£ Ω	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //—°‘Ò∂® ±∆˜ƒ£ Ω:TIM¬ˆ≥ÂøÌ∂»µ˜÷∆ƒ£ Ω1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //±»Ωœ ‰≥ˆ πƒ‹
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // ‰≥ˆº´–‘:TIM ‰≥ˆ±»Ωœº´–‘µÕ
	TIM_OC1Init(TIM12, &TIM_OCInitStructure);  //∏˘æ›T÷∏∂®µƒ≤Œ ˝≥ı ºªØÕ‚…ËTIM12 4OC2

	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);  // πƒ‹TIM124‘⁄CCR2…œµƒ‘§◊∞‘ÿºƒ¥Ê∆˜
 
  TIM_ARRPreloadConfig(TIM12,ENABLE);//ARPE πƒ‹ 
	
	TIM_Cmd(TIM12, ENABLE);  // πƒ‹TIM2								  
}  



