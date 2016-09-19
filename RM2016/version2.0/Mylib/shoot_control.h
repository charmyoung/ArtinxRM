#ifndef _SHOOT_CONTROL_H_
#define _SHOOT_CONTROL_H_
#include "stdint.h"

extern int rub_flag;//s2-1位开启状态
extern int shoot_flag;//s2-2位开启状态

void ShootMotor_Velocity_Control(float TargetShootSpeed);
void BLDC_control(uint8_t s2, uint8_t press_r);
void Fire(uint8_t s2, uint8_t press_l);

#endif
