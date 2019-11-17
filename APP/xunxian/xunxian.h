#ifndef __xunxian_H
#define __xunxian_H	 
#include "sys.h"
#include "usart.h"	
#include "main.h"
u8 xundo(void);
u8 startxun(void);
void xunxian_Init(void);
void scanxunxian(void);
void judgex(void);
void judgey(void);
u8 judgesz(void);
extern int vy;
extern int dx;
#define HC165_OUT1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)//读取按键0
#define HC165_nPL1    PDout(6)
#define HC165_CK1     PGout(9)
#define HC165_INH1     PGout(15)

#define HC165_OUT2  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_7)//读取按键0
#define HC165_nPL2    PDout(11)
#define HC165_CK2    PGout(3)
#define HC165_INH2     PGout(5)


#define HC165_OUT3  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_0)//读取按键0
#define HC165_nPL3    PGout(1)
#define HC165_CK3    PFout(15)
#define HC165_INH3     PFout(13)

#define HC165_OUT4 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)
#define HC165_nPL4 PFout(14)
#define HC165_CK4  PAout(4)
#define HC165_INH4     PAout(1)


#define HC165_OUT5 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_3)
#define HC165_nPL5 PFout(9)
#define HC165_CK5   PFout(7)
#define HC165_INH5     PFout(5)

#endif




/*dx置1时开始y矫正，dx置0时关闭*/

