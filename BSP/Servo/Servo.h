#ifndef __SERVO_H
#define __SERVO_H
#include "sys.h"



void Servo_Task(void);
void Servo_Task_init(void);
//void TIM1_CH1_PWM_Init(u32 arr,u32 psc);
void TIM3_CH3_PWM_Init(u32 arr,u32 psc);
void TIM9_CH2_PWM_Init(u32 arr,u32 psc);
void TIM12_CH1_PWM_Init(u32 arr,u32 psc);
#endif
