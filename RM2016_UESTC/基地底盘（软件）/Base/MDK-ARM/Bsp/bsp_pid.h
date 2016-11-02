#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "stm32f4xx_hal.h"

//取绝对值
#define ABS(x)		((x>0)?x:-x)

//PID的ID的枚举
typedef enum
{
  PID_ANGLE = 0,
  PID_VX,
  PID_VY,
  PID_EGGROTATE_ANG,
  PID_EGGROTATE_OMG,
//	Gimbal_PID_Pitch = 0,
//	Gimbal_PID_Yaw,
//	Gimbal_PID_PitchSpeed,
//	Gimbal_PID_YawSpeed,
//	
//	UnderPan_PID_Yaw,
}PID_ID;

enum
{
  DB_OUTPUT_LAST = 0,
  DB_OUTPUT_ZERO = 1,
};


//PID结构体
typedef struct _PID_TypeDef
{
	PID_ID id;
	
	float target;							//目标值
	
	float kp;
	float ki;
	float kd;
	
	float   measure;					//测量值
	float   err;							//误差
	float   last_err;      		//上次误差
	
	float   pout;							//p输出
	float   iout;							//i输出
	float   dout;							//d输出
	
	float   output;						//本次输出
	float   last_output;			//上次输出
	
	uint16_t MaxOutput;				//输出限幅
	uint16_t IntegralLimit;		//积分限幅
	uint16_t DeadBand;			  //死区（绝对值）
	uint16_t ControlPeriod;		//控制周期
	int16_t  Max_Err;					//最大误差(这东西坑了好多人）
  
  uint8_t  DeadBandOpType;  //在死区时的输出类型
	
	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
					PID_ID id,
					uint16_t maxOutput,
				  uint16_t integralLimit,
					uint16_t deadband,
					uint16_t controlPeriod,
					int16_t	 max_err,     
					float    target,
				   
					float kp,
					float ki,
					float kd,
            
          uint8_t deadbandOpType);
				   
	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid三个参数修改
	int16_t (*f_cal_pid)(struct _PID_TypeDef *pid, float target, float measure);  //pid计算
}PID_TypeDef;

//函数声明
void PID_Configuration(
	PID_TypeDef* pid,
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	uint16_t deadband,
	uint16_t period,
	int16_t  max_err,
	float    target,

	float 	kp, 
	float 	ki, 
	float 	kd,
    
  uint8_t deadbandOpType);
#endif
