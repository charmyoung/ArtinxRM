#ifndef _CHASSIS_CONTROL_H_
#define _CHASSIS_CONTROL_H_
#include "stdint.h"
//
typedef struct _RM35_DATA_
{
	short thisCurrent;
	short thisVelocity;
	long  thisPosition;
	char Online;
	char Ctl1_Value;
	char Ctl2_Value;
}RM35_DATA;
extern  RM35_DATA RM35_1;
extern  RM35_DATA RM35_2;
extern  RM35_DATA RM35_3;
extern  RM35_DATA RM35_4;

typedef struct _RM3510_DATA_
{
	//从电调反馈读到的数值	
	int16_t thisPosition;//处理后的反馈角度
	int16_t thisVelocity;//反馈电流
	//目标设定数值
	int16_t targetPosition;//目标角度
	int16_t targetVelocity;//目标速度
	//PID
	float position_output;//位置环输出，位置环输入
	float velocity_output;//速度环输出，电流环输入
	//编号值，目前无用
	uint8_t number;
	
}RM3510_DATA;

extern RM3510_DATA RM3510_1;
extern RM3510_DATA RM3510_2;
extern RM3510_DATA RM3510_3;
extern RM3510_DATA RM3510_4;

//3510用 
void ChassisMotor_Velocity_Control(float vel1,float vel2,float vel3,float vel4);
void ChassisMotor_Position_Control(float pos1,float pos2,float pos3,float pos4);

extern int moveSpeed;
void TransMove(int x,int y,int z,long temp_speed);
void move_control(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint8_t s1, uint16_t v,int16_t x);

#endif
