#include "main.h"

uint32_t missing_counter[MISSING_COUNTER_NUM] = { 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000,2000};
const uint32_t missing_counter_len[MISSING_COUNTER_NUM] = {1000, 1000, 1000, 1000, 1000, 1000, 200, 1000, 1000, 1000, 1000};
uint32_t lost_err = 0xFFFFFFFF;     //每一位代表一个错误
u8 hardware_error[MISSING_COUNTER_NUM];



void LCD_PREPARE()
{
//	LCD_Init();           //初始化LCD FSMC接口
//	LCD_Clear(WHITE);
//	POINT_COLOR=RED;  
//	LCD_ShowString(30,40,250,24,24,"Initializing");	
//	LCD_ShowString(30,80,250,24,24,"LIDAR:UNKNOWN");	
//	LCD_ShowString(30,120,250,24,24,"GYRO:UNKNOWN");
//	LCD_ShowString(30,160,250,24,24,"BASE_MOTOR1:UNKNOWN");
//	LCD_ShowString(30,200,250,24,24,"BASE_MOTOR2:UNKNOWN");
//	LCD_ShowString(30,240,250,24,24,"BASE_MOTOR3:UNKNOWN");		//显示LCD ID	      					 
//	LCD_ShowString(30,280,250,24,24,"THROW_MOTOR1:UNKNOWN");	
//	LCD_ShowString(30,320,250,24,24,"THROW_MOTOR2:UNKNOWN");	
}


void Supervise()
{
	int i =0;
		//step 1;check the hardware_error
	for(i = 0; i < MISSING_COUNTER_NUM; i++)
	{
		if(MissingCounterOverflowCheck(missing_counter[i], missing_counter_len[i]) == 0)
		{
			MissingCounterCount(&missing_counter[i], 1);   //add 1 everytime
			lost_err &= ~(uint32_t)(1 << i);    //clear the hardware_error bit; 0 represents noth wrong
			hardware_error[i]=0;                          //clear the hardware_error
		}
		else
		{
			lost_err |= (uint32_t)(1 << i);    //set the hardware_error bit; 1 represents sothing wrong
			hardware_error[i]=1;                     //set the hardware_error
		}
		
	}
	
		static u16 shunxu=0;
		shunxu++;
		if(shunxu<100)
		{
			PAGE0_TXT0(ROBOT_POSITION.X_POS,ROBOT_POSITION.Y_POS,ROBOT_POSITION.ANGLE_POS,LIDAR_POSITION.X_POS,LIDAR_POSITION.Y_POS,
								LIDAR_POSITION.ANGLE_POS,Encoder.ENCONDER_POS_A,Encoder.ENCONDER_POS_B,ROBOT_STATE.Vx,ROBOT_STATE.Vy,ROBOT_STATE.Vz);
		}
		
		else if(shunxu>=100 && shunxu<200)
		{
			PAGE0_TXT1(hardware_error[6],hardware_error[0],hardware_error[4],hardware_error[5],hardware_error[1],
			           hardware_error[2],hardware_error[3],hardware_error[6],hardware_error[6],hardware_error[6]);
		}
		else if(shunxu>=200 && shunxu<300) 
		{
			PAGE1_TXT0(1,1,1,1,1,
								1,1,1,1,1);
		}
		else if(shunxu>=300)  
			shunxu=0;
		
	
	
	
	//step 2:action to error 	

}



//uint32_t *GetMissingCounter(uint8_t index)
//{
//    if(index < MISSING_COUNTER_NUM)
//    {
//        return &missing_counter[index];
//    }
//    else
//    {
//        return ((void *)0);
//    } 
//}


void MissingCounterCount(uint32_t *mc, uint32_t os_ticks)
{
	*mc += os_ticks;
}

void LostCounterFeed(uint32_t *mc)
{
	*mc = 2000;
}
//MissingCounterFeed(GetLostCounter(MISSING_COUNTER_INDEX_MOTOR2));

uint8_t MissingCounterOverflowCheck(uint32_t mc, uint32_t threshold)
{
	if(mc - 2000 > threshold)
		return 1;
	else
		return 0;
}

//return the error code
uint32_t Get_Lost_Error(uint32_t err_code)
{
	return lost_err&err_code;
}


void senddata(float a)
{
		u8 temp4[6];
	  sprintf(temp4,"%f",a);
		u8 len4=strlen(temp4);
		for(int i=0;i<len4;i++)
		{
			USART_SendData(USART1, temp4[i]);         //向串口2发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}		
		u8 temp2[]="\r\n";
		u8 len2=strlen(temp2);
		for(int i=0;i<len2;i++)
		{
			USART_SendData(USART1, temp2[i]);         //向串口2发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		}

}

void PAGE0_TXT0(float fir,float sec,float thir,float fort,float fif,
								float six,float sev,float eigh,float nin,float ten,float ele)
{
//		u8 temp[]="page0.t0.txt=\"";    //第一页第一个文本框
//		u8 len=strlen(temp);
//		for(int i=0;i<len;i++)
//		{
//			USART_SendData(USART1, temp[i]);         //向串口2发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}
//		
//		senddata(fir);
//		senddata(sec);
//		senddata(thir);
//		senddata(fort);
//		senddata(fif);
//		senddata(six);
//		senddata(sev);
//		senddata(eigh);
//		senddata(nin);
//		senddata(ten);


//		u8 temp2[]="\"";
//		u8 len2=strlen(temp2);
//		for(int i=0;i<len2;i++)
//		{
//			USART_SendData(USART1, temp2[i]);         //向串口2发送数据
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}
//		for(int t=0;t<3;t++)
//		{
//			USART_SendData(USART1, 0XFF);
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//		}
//		
//		




	  static u8 order=0;
		static u8 temp[]="page0.t0.txt=\"";    //第一页第一个文本框
		u8 len=strlen(temp);
		static u8 temp2[]="\"";
		u8 len2=strlen(temp2);
		switch(order)
		{
			case 1:{	
									for(int i=0;i<len;i++)
									{
										USART_SendData(USART1, temp[i]);         //向串口2发送数据
										while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
									}
						}break;
			case 2:{senddata(fir);}break;
			case 3:{senddata(sec);}break;
			case 4:{senddata(thir);}break;
			case 5:{senddata(fort);}break;
			case 6:{senddata(fif);}break;
			case 7:{senddata(six);}break;
			case 8:{senddata(sev);}break;
			case 9:{senddata(eigh);}break;
			case 10:{senddata(nin);}break;
			case 11:{senddata(ten);}break;
			case 12:{senddata(ele);}break;
			case 13:{
								for(int i=0;i<len2;i++)
								{
									USART_SendData(USART1, temp2[i]);         //向串口2发送数据
									while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
								}
							}break;			
			case 14:{
								for(int t=0;t<3;t++)
								{
									USART_SendData(USART1, 0XFF);
									while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
								}
							}break;			

		}
			order++;
			if(order>100) order = 0;
}

void PAGE0_TXT1(float fir,float sec,float thir,float fort,float fif,
								float six,float sev,float eigh,float nin,float ten)
{
	  static u8 order=0;
		static u8 temp[]="page0.t2.txt=\"";    //第一页第一个文本框
		u8 len=strlen(temp);
		static u8 temp2[]="\"";
		u8 len2=strlen(temp2);
		switch(order)
		{
			case 1:{	
									for(int i=0;i<len;i++)
									{
										USART_SendData(USART1, temp[i]);         //向串口2发送数据
										while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
									}
						}break;
			case 2:{senddata(fir);}break;
			case 3:{senddata(sec);}break;
			case 4:{senddata(thir);}break;
			case 5:{senddata(fort);}break;
			case 6:{senddata(fif);}break;
			case 7:{senddata(six);}break;
			case 8:{senddata(sev);}break;
			case 9:{senddata(eigh);}break;
			case 10:{senddata(nin);}break;
			case 11:{senddata(ten);}break;
			case 12:{
								for(int i=0;i<len2;i++)
								{
									USART_SendData(USART1, temp2[i]);         //向串口2发送数据
									while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
								}
							}break;			
			case 13:{
								for(int t=0;t<3;t++)
								{
									USART_SendData(USART1, 0XFF);
									while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
								}
							}break;			

		}
			order++;
			if(order>100) order = 0;
		
}

void PAGE1_TXT0(float fir,float sec,float thir,float fort,float fif,
								float six,float sev,float eigh,float nin,float ten)
{
	  static u8 order=0;
		static u8 temp[]="page1.t1.txt=\"";    //第一页第一个文本框
		u8 len=strlen(temp);
		static u8 temp2[]="\"";
		u8 len2=strlen(temp2);
		switch(order)
		{
			case 1:{	
									for(int i=0;i<len;i++)
									{
										USART_SendData(USART1, temp[i]);         //向串口2发送数据
										while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
									}
						}break;
			case 2:{senddata(fir);}break;
			case 3:{senddata(sec);}break;
			case 4:{senddata(thir);}break;
			case 5:{senddata(fort);}break;
			case 6:{senddata(fif);}break;
			case 7:{senddata(six);}break;
			case 8:{senddata(sev);}break;
			case 9:{senddata(eigh);}break;
			case 10:{senddata(nin);}break;
			case 11:{senddata(ten);}break;
			case 12:{
								for(int i=0;i<len2;i++)
								{
									USART_SendData(USART1, temp2[i]);         //向串口2发送数据
									while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
								}
							}break;			
			case 13:{
								for(int t=0;t<3;t++)
								{
									USART_SendData(USART1, 0XFF);
									while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
								}
							}break;			

		}
			order++;
			if(order>100) order = 0;
		
}













//void LCD_TASK()
//{
//	static int LCD_PAGE = 1;
//	static int LCD_PAGE_count = 0;
//	LCD_PAGE_count++;
//	if(LCD_PAGE_count>30)
//		LCD_PAGE = 1;
//	if(LCD_PAGE_count>33)
//	{
//		LCD_PAGE = 2;
//		LCD_PAGE_count = 0;
//	}
//	switch(LCD_PAGE)
//	{
//		case 1:LCD_PAGE1_SENSORS();break;
//		case 2:LCD_PAGE2_ROBOTSTATE();break;
//		case 3:LCD_PAGE3_TRACKING();break;
//		default:{}
//	}
//}




//void LCD_PAGE1_SENSORS()
//{
//	//MOTOR,LIDAR,GYRO
//	LCD_Clear(WHITE);
//	POINT_COLOR=RED;
//	
//	if(Get_Lost_Error(MISSING_ERROR_GYRO) == MISSING_ERROR_GYRO)
//	{
//		LCD_ShowString(30,80,350,24,24,"GYRO: GG");
//	}
//	else
//	{
//		LCD_ShowString(30,80,350,24,24,"GYRO: NO PRO");
//	}
//	
//	
//	if(Get_Lost_Error(MISSING_ERROR_LIDAR) == MISSING_ERROR_LIDAR)
//	{
//		LCD_ShowString(30,120,350,24,24,"LIDAR: GG");
//	}
//	else
//	{
//		LCD_ShowString(30,120,350,24,24,"LIDAR: NO PRO");
//	}
//	
//	/********************************************************/
//	if(Get_Lost_Error(MISSING_ERROR_MOTOR4) == MISSING_ERROR_MOTOR4)
//	{
//		LCD_ShowString(30,280,350,24,24,"THROW_MOTOR1: GG");
//	}
//	else
//	{
//		LCD_ShowString(30,280,350,24,24,"THROW_MOTOR1: NO PRO");
//	}
//	/********************************************************/
//	
//	if(Get_Lost_Error(MISSING_ERROR_MOTOR5) == MISSING_ERROR_MOTOR5)
//	{
//		LCD_ShowString(30,320,350,24,24,"THROW_MOTOR2: GG");
//	}
//	else
//	{
//		LCD_ShowString(30,320,350,24,24,"THROW_MOTOR2: NO PRO");
//	}
//	/********************************************************/
//	
//	if(Get_Lost_Error(MISSING_ERROR_MOTOR1) == MISSING_ERROR_MOTOR1)
//	{
//		LCD_ShowString(30,160,350,24,24,"BASE_MOTOR1: GG");
//	}
//	else
//	{
//		LCD_ShowString(30,160,350,24,24,"BASE_MOTOR1:NO PRO");
//	}
//	/********************************************************/
//	
//	if(Get_Lost_Error(MISSING_ERROR_MOTOR2) == MISSING_ERROR_MOTOR2)
//	{
//		LCD_ShowString(30,200,350,24,24,"BASE_MOTOR2: GG");
//	}
//	else
//	{
//		LCD_ShowString(30,200,350,24,24,"BASE_MOTOR2: NO PRO");
//	}
//	
//	/********************************************************/
//	if(Get_Lost_Error(MISSING_ERROR_MOTOR3) == MISSING_ERROR_MOTOR3)
//	{
//		LCD_ShowString(30,240,350,24,24,"BASE_MOTOR3: GG");	
//	}
//	else
//	{
//		LCD_ShowString(30,240,350,24,24,"BASE_MOTOR3: NO PRO");	
//	}
//	/********************************************************/
//	
//	if(Get_Lost_Error(MISSING_ERROR_DEADLOCK) == MISSING_ERROR_DEADLOCK)
//	{
//		
//	}
//	else
//	{
//		
//	}
//	
////	LCD_ShowString(30,40,350,24,24,"NOWSTATE:");	
////	LCD_ShowString(30,80,350,24,24,"LIDAR: GG");	
////	LCD_ShowString(30,120,350,24,24,"GYRO: GG");
////	LCD_ShowString(30,160,350,24,24,"BASE_MOTOR1: GG");
////	LCD_ShowString(30,200,350,24,24,"BASE_MOTOR2: GG");
////	LCD_ShowString(30,240,350,24,24,"BASE_MOTOR3: GG");    					 
////	LCD_ShowString(30,280,350,24,24,"THROW_MOTOR1: GG");	
////	LCD_ShowString(30,320,350,24,24,"THROW_MOTOR2: GG");
////	LCD_ShowString(30,360,250,24,24,"VX:");
////	LCD_ShowString(30,400,250,24,24,"VY:");
////	LCD_ShowString(30,440,250,24,24,"VZ:");
//	
//}
//void LCD_PAGE2_ROBOTSTATE()
//{
//	char buf[20];
//	LCD_Clear(WHITE);
//	POINT_COLOR=RED;  
//	LCD_ShowString(30,40,350,24,24,"NOWSTATE:");	
//	
//	LCD_ShowString(30,80,250,24,24,"POSITION:");
//	
//	LCD_ShowString(30,120,350,24,24,"X:");
//	sprintf(buf, "%f",ROBOT_POSITION.X_POS);
//	LCD_ShowString(30+100,120,150,24,24,	buf);
//	
//	LCD_ShowString(30,160,350,24,24,"Y:");
//	sprintf(buf, "%f",ROBOT_POSITION.Y_POS);
//	LCD_ShowString(30+100,160,150,24,24,	buf);
//	
//	LCD_ShowString(30,200,350,24,24,"ANGLE:");
//	sprintf(buf, "%f",ROBOT_POSITION.ANGLE_POS);
//	LCD_ShowString(30+100,200,150,24,24,	buf);
//	
//	LCD_ShowString(30,240,350,24,24,"ENCODERA:");		//显示LCD ID	
//	sprintf(buf, "%ld",Encoder.ENCONDER_POS_A);
//	LCD_ShowString(30+150,240,150,24,24,	buf);
//	LCD_ShowString(30,280,350,24,24,"ENCODERB:");	
//	sprintf(buf, "%ld",Encoder.ENCONDER_POS_B);
//	LCD_ShowString(30+150,280,150,24,24,	buf);
//	
//	LCD_ShowString(30,320,250,24,24,"SPEED:");
//	
//	LCD_ShowString(30,360,250,24,24,"VX:");
//	sprintf(buf, "%d",ROBOT_STATE.Vx);
//	LCD_ShowString(30+100,360,150,24,24,	buf);
//	
//	LCD_ShowString(30,400,250,24,24,"VY:");
//	sprintf(buf, "%d",ROBOT_STATE.Vy);
//	LCD_ShowString(30+100,400,150,24,24,	buf);
//	
//	LCD_ShowString(30,440,250,24,24,"VZ:");
//	sprintf(buf, "%d",ROBOT_STATE.Vz);
//	LCD_ShowString(30+100,440,150,24,24,	buf);
//}
//void LCD_PAGE3_TRACKING()
//{
//	char buf[20];
//	LCD_Clear(WHITE);
//	POINT_COLOR=RED;  
//	LCD_ShowString(30,40,350,24,24,"NOWSTATE:");	
//	
//	LCD_ShowString(30,80,250,24,24,"POSITION:");
//	
//	LCD_ShowString(30,120,350,24,24,"X:");
//	sprintf(buf, "%f",ROBOT_POSITION.X_POS);
//	LCD_ShowString(30+100,120,150,24,24,	buf);
//	
//	LCD_ShowString(30,160,350,24,24,"Y:");
//	sprintf(buf, "%f",ROBOT_POSITION.Y_POS);
//	LCD_ShowString(30+100,160,150,24,24,	buf);
//	
//	LCD_ShowString(30,200,350,24,24,"ANGLE:");
//	sprintf(buf, "%f",ROBOT_POSITION.ANGLE_POS);
//	LCD_ShowString(30+100,200,150,24,24,	buf);
//	
//	LCD_ShowString(30,240,350,24,24,"ENCODERA:");		//显示LCD ID	
//	sprintf(buf, "%ld",Encoder.ENCONDER_POS_A);
//	LCD_ShowString(30+150,240,150,24,24,	buf);
//	LCD_ShowString(30,280,350,24,24,"ENCODERB:");	
//	sprintf(buf, "%ld",Encoder.ENCONDER_POS_B);
//	LCD_ShowString(30+150,280,150,24,24,	buf);
//	
//	LCD_ShowString(30,320,250,24,24,"SPEED:");
//	
//	LCD_ShowString(30,360,250,24,24,"VX:");
//	sprintf(buf, "%d",ROBOT_STATE.Vx);
//	LCD_ShowString(30+100,360,150,24,24,	buf);
//	
//	LCD_ShowString(30,400,250,24,24,"VY:");
//	sprintf(buf, "%d",ROBOT_STATE.Vy);
//	LCD_ShowString(30+100,400,150,24,24,	buf);
//	
//	LCD_ShowString(30,440,250,24,24,"VZ:");
//	sprintf(buf, "%d",ROBOT_STATE.Vz);
//	LCD_ShowString(30+100,440,150,24,24,	buf);
//}
