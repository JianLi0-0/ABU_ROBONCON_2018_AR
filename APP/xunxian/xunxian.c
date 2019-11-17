#include "xunxian.h"


int vx=0,vy=0,vz=0,dx=0;
u8 cdata[8] = {0};
u8 odata[8]= {0};
u8 xdata[8]= {0};
int a1,a2,b1,c1,cx,cd;

int h,j;
/*start,若完成返回1*/
u8 xundo(void)
{ 

	if(dx==0)
	{
		HC165_nPL1 = 0;        
		 HC165_nPL1 = 1;        
					cdata[0]=HC165_OUT1;
		 for(h = 1;h < 8;h++)  
		{    
				HC165_CK1 = 0;  
				HC165_CK1 = 1;      
				cdata[h]=HC165_OUT1 ;
		}
			for(h = 0;h < 8;h++)
		{ 
			if(cdata[h]==0)
			 c1=h+1;			
		}
	}
	if(dx==1)
	{
	vy=0;
	scanxunxian();
	startxun();
	}
	return 0;
//	}
}

/*寻线初始化*/
void xunxian_Init(void)
	
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG,ENABLE);
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_11;//LED0和LED1对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//xiala
	
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO		
  GPIO_SetBits(GPIOD,GPIO_Pin_6|GPIO_Pin_11);						 //PA.8 输出高

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_9 | GPIO_Pin_15;//LED0和LED1对应IO口	
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOG,GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_9 | GPIO_Pin_15);						 //PA.8 输出高

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_7| GPIO_Pin_9|GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15 ;//LED0和LED1对应IO口	
	GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_SetBits(GPIOF,GPIO_Pin_5| GPIO_Pin_7| GPIO_Pin_9|GPIO_Pin_13|GPIO_Pin_15);						 //PA.8 输出高
	
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ;//LED0和LED1对应IO口	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_4);	



	 HC165_INH1=0;
	 HC165_INH2=0;
   HC165_INH3=0;
	 HC165_INH4=0;
	 HC165_INH5=0;


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_7; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIOE2,3,4
}

/*读取传感器值*/
void scanxunxian(void)
	

{
	
	HC165_nPL5 = 0;        
     HC165_nPL5 = 1;        
					xdata[0]=HC165_OUT5;
     for(h = 1;h < 8;h++)  
    {    
        HC165_CK5 = 0;  
        HC165_CK5 = 1;      
        xdata[h]=HC165_OUT5 ;
    }
		
		
		 HC165_nPL4 = 0;        
     HC165_nPL4 = 1;    
					odata[0]=HC165_OUT4;
    
	  for(h = 1;h < 8;h++)  
    {    
        HC165_CK4 = 0;  
        HC165_CK4 = 1;      
        odata[h]=HC165_OUT4 ;


    }
		
		HC165_nPL1 = 0;        
     HC165_nPL1 = 1;        
					cdata[0]=HC165_OUT1;
     for(h = 1;h < 8;h++)  
    {    
        HC165_CK1 = 0;  
        HC165_CK1 = 1;      
        cdata[h]=HC165_OUT1 ;
    }
 /* cdata[0]=odata[1];
	cdata[1]=odata[0];
  cdata[2]=odata[2];
  cdata[3]=odata[3];
  cdata[4]=odata[4];
  cdata[5]=odata[5];
  cdata[6]=odata[6];
  cdata[7]=odata[7];*/


		
/*	printf("%d",xdata[0]);
	printf("%d",xdata[1]);
			printf("%d",xdata[2]);
			printf("%d",xdata[3]);
	printf("%d",xdata[4]);
	printf("%d",xdata[5]);
	printf("%d",xdata[6]);
	printf("%d\n",xdata[7]);*/
		/*printf("%d",cdata[0]);

		printf("%d",cdata[1]);

  printf("%d",cdata[2]);
	printf("%d",cdata[3]);
	printf("%d",cdata[4]);
	printf("%d",cdata[5]);
	printf("%d",cdata[6]);
	printf("%d\n",cdata[7]);

	delay_ms(10);*/
	cx=c1;
  c1=0;
		  for(h = 0;h < 8;h++)
    { 
			if(cdata[h]==0)
			 c1=h+1;			
	  }
		cd=abs(cx-c1);
		if(cd>=2)
			c1=cx;
}
/*开始寻线*/
u8 startxun(void)
{
  if(c1==4||c1==5)
		vy=0;
	if(c1==1)
		vy=800;
	if(c1==2)
		vy=650;
	if(c1==3)
		vy=500;
	if(c1==8)
		vy=-800;
	if(c1==7)
		vy=-650;
	if(c1==6)
		vy=-500;
	
	  return 0;
}

