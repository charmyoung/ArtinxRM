#ifndef __CAN2_H__
#define __CAN2_H__
#include "stdint.h"

#ifdef ROBOMODULE_NEW
	#define PWM_MODE                                0x01
	#define PWM_CURRENT_MODE                        0x02
	#define PWM_VELOCITY_MODE                       0x03
	#define PWM_POSITION_MODE                       0x04
	#define PWM_VELOCITY_POSITION_MODE              0x05
	#define CURRENT_VELOCITY_MODE                   0x06
	#define CURRENT_POSITION_MODE                   0x07
	#define CURRENT_VELOCITY_POSITION_MODE          0x08
#endif

#ifdef ROBOMODULE_OLD
	#define PWM_MODE                                0x11
	#define PWM_CURRENT_MODE                        0x22
	#define PWM_VELOCITY_MODE                       0x33
	#define PWM_POSITION_MODE                       0x44
#endif


void CAN2_Configuration(void);

//RM3510
void Cmd_ESC_820R(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);

//RM35
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number);
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode);
void CAN_RoboModule_DRV_PWM_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);
void CAN_RoboModule_DRV_PWM_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current);
void CAN_RoboModule_DRV_PWM_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
void CAN_RoboModule_DRV_PWM_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position);
void CAN_RoboModule_DRV_PWM_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity);
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position);
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2);
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number);

#endif 
