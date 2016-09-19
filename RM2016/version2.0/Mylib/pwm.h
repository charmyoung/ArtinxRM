#ifndef __PWM_H__
#define __PWM_H__
#include  "stm32f4xx.h"
void PWM_Configuration(void);
void ShootMotorSpeedSet(s32 MotorSpeed);
#define PWM1  TIM5->CCR1
#define PWM2  TIM5->CCR2
#define PWM3  TIM9->CCR1


#endif
