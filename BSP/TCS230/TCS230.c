#include "main.h"


u32 Colour_num=0;								  // 用于颜色传感器脉冲计数
u8  Time7_flag=0;                 // 定时器中断标志

////////////////////////////////////////////////////////////////////////////////////

/*==============================以下为定时器3程序部分=================================*/
//通用定时器7中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器7!
void TIM7_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///使能TIM7时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器7更新中断
	TIM_Cmd(TIM7,ENABLE); //使能定时器7
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //定时器7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/*定时器7中断服务函数*/
//计时溢出清除
void TIM7_IRQHandler(void)
{ 
 
  if(TIM7->SR&0X0001)//溢出中断
   {
	 Time7_flag=1;
	 EXTI->IMR|=0<<0;//关闭外部中断0
	 TIM7->CR1|=0X00;	//关闭定时器7
   }
   TIM7->SR&=0<<0;//清除中断标志位 

}
/////////////////////////////////////////////////////////////////////////////////////


/*=============================以下为外部中断0程序部分=================================*/
//外部中断初始化程序
//初始化PB0为中断输入.用于颜色传感器的out输入计数
void TCS230_EXTIX_Init(void)
{

	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
   

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);//PB0 连接到中断线0

  /* 配置EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
  EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
}

/*外部中断0服务程序*/
void EXTI0_IRQHandler(void)
{
	Colour_num++;
  EXTI->PR = 1<<0; //清除line0上的中断标志位
	EXTI->IMR|=0<<0;//关闭外部中断0
	TIM7->CR1|=0X00;	//关闭定时器7
}

/**********************************************************
 函数名称：TCS230_Init
 函数功能：TCS230初始化
 入口参数：无
 返回参数：无
 备    注：初始化外部中断0及定时器中断3及相关I/O口设置
**********************************************************/ 
void TCS230_Init(void)
{
 	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟
		 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // TCS230输出引脚out对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//浮空
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB0
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; // TCS230输出引脚S2,S3对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE0,GPIOE1
	
	
	TIM7_Int_Init(500,8400);//50ms
	TCS230_EXTIX_Init();
	
}	
	
/***********************************************
 函数名称：Colour_Init
 函数功能：TCS230内部颜色滤波器设置
 入口参数：Colour
 返回参数：无
 备    注：Colour 选择颜色（Red1/Blue1/Green1）
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
 函数名称：TCS230_Start
 函数功能：初始化各变量及定时器3、外部中断0，用于脉冲计数
 入口参数：无
 返回参数：无
**********************************************************/ 
void TCS230_Start(void)
{		
    TIM7->CR1|=0X01;	//使能定时器7
		Colour_num=0;//	计数清零
		Time7_flag=0; // 定时器标志清零
	  EXTI->IMR|=1<<0;//开启外部中断0	????????????????????
}



/******************************************
 函数名称：TCS230_Read_Colour
 函数功能：设置颜色滤波器并读出相应频率参数
 入口参数：Colour
 返回参数：Colour num
 备    注：Colour     ：颜色滤波器配置参数
           Colour num ：脉冲个数
******************************************/ 
u32 TCS230_Read_Colour(u8 Colour)
{
	Colour_Init(Colour);//配置TCS230的颜色滤波器
	delay_ms(35);
	TCS230_Start();//初始化各变量及定时器3、外部中断0
	while(Time7_flag==0);//等待计数完毕
	return Colour_num;	// 返回计数个数
}



/******************************************
 函数名称：TCS230_Distinguish
 函数功能：物体颜色判别
 入口参数：    
 返回参数：1/2/3
	         flag_color=r,i=1;red;
	         flag_color=g,i=2;green;
	         flag_color=b;i=3;blue;
 备    注：0:Red1
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














