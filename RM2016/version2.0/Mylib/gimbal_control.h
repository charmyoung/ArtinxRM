#ifndef __GIMBAL_CONTROL_H__
#define __GIMBAL_CONTROL_H__
#include "stdint.h"

typedef struct _M6623_
{
	int16_t minAngle;//最小角度
	int16_t maxAngle;//最大角度
	int16_t defualtAngle;//初始角度
	
	//从云台中读到的数值
	int16_t thisAngle_Raw;//原生的反馈角度
	int16_t thisAngle;//处理后的反馈角度
	int16_t thisCurrent;//反馈电流
	int16_t targetCurrent;//目标电流
	
	int16_t targetAngle;//目标角度
	
	float position_output;//位置环输出，位置环输入
	float velocity_output;//速度环输出，电流环输入

}M6623;

extern M6623 Yaw,Pitch;

void Gimbal_Control(void);
void Trigger_Control(int16_t x,int16_t y,uint16_t ch3);

#endif
