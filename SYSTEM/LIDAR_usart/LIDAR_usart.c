#include "main.h"
float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;

lidar LIDAR_POS;

void USART3_IRQHandler(void)                	//串口1中断服务程序
{
	static uint8_t ch;
	static union
	{
	 uint8_t data[24];
	 float ActVal[6];
	}posture;
	static uint8_t count=0;
  static uint8_t i=0;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)   
	{
		USART_ClearITPendingBit( USART3,USART_IT_RXNE);	
		ch=USART_ReceiveData(USART3);
		
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
//	    


		 
	 }
}


void Action_uart_init(u32 bound)
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA9复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA10复用为USART3
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、


}


void send_pos(float pos_x, float pos_y, float zangle)
{
	u8 t;
	static union
	{
	 uint8_t data[24];
	 float ActVal[6];
	}posture;

	posture.ActVal[0] = zangle;
	posture.ActVal[3] = pos_x;
	posture.ActVal[4]	= pos_y;

	USART_SendData(USART3, 0x0d);         //???1????
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//??????
	USART_SendData(USART3, 0x0a);         //???1????
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//??????
	
	for(t=0;t<24;t++)
	{
		USART_SendData(USART3, posture.data[t]);         //???1????
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//??????
	}
	
	USART_SendData(USART3, 0x0a);         //???1????
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//??????
	USART_SendData(USART3, 0x0d);         //???1????
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//??????

}



