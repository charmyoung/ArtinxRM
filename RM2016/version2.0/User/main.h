#ifndef __MAIN_H__
#define __MAIN_H__

//定义车号,主要对应电机和云台
#define CAR_A
//定义自动车
#define AUTO_TRACK
//CAR1 RM35-ROBOMODULE_NEW
//CAR2 RM35-ROBOMODULE_NEW
//CAR3 RM3510

//定义工作与调试模式
//#define DEBUG



//根据车号定义
#ifdef CAR1
//定义底盘电机和驱动板
#define ROBOMODULE_NEW

//云台角度
#define YAW_LEFT 2900
#define YAW_RIGHT	5400
#define YAW_MID 4100
#define PITCH_UP 2600
#define PITCH_DOWN 1800
#define PITCH_MID 2100

//云台PID
#define YAW_205_VEL_P 20.0
#define YAW_205_VEL_D 0.0
#define YAW_205_POS_P 3.5
#define YAW_205_POS_I 0.0
#define YAW_205_POS_D 0.1
#define PITCH_206_VEL_P 15.0
#define PITCH_206_VEL_D 5.0
#define PITCH_206_POS_P 2.0
#define PITCH_206_POS_I 0.0
#define PITCH_206_POS_D 0.0
//定义拨弹电机速度
#define SHOOT_SPEED  10   //0-10000
#endif

#ifdef CAR2 
//定义底盘电机和驱动板
#define ROBOMODULE_NEW

//云台角度
#define YAW_LEFT 1450
#define YAW_RIGHT	5550
#define YAW_MID 3400
#define PITCH_UP 3250
#define PITCH_DOWN 2250
#define PITCH_MID 2500
//云台PID
#define YAW_205_VEL_P 20.0
#define YAW_205_VEL_D 0.0
#define YAW_205_POS_P 2.0
#define YAW_205_POS_I 0.0
#define YAW_205_POS_D 0.1
#define PITCH_206_VEL_P 20.0
#define PITCH_206_VEL_D 0.0
#define PITCH_206_POS_P 1.0
#define PITCH_206_POS_I 0.0
#define PITCH_206_POS_D 0.0

#endif

#ifdef CAR3 
//定义底盘电机
#define RM3510 

//云台角度
#define YAW_LEFT 1450
#define YAW_RIGHT	5550
#define YAW_MID 3400
#define PITCH_UP 3250
#define PITCH_DOWN 2050
#define PITCH_MID 2500
//云台PID
#define YAW_205_VEL_P 20.0
#define YAW_205_VEL_D 0.0
#define YAW_205_POS_P 2.0
#define YAW_205_POS_I 0.0
#define YAW_205_POS_D 0.1
#define PITCH_206_VEL_P 20.0
#define PITCH_206_VEL_D 0.0
#define PITCH_206_POS_P 1.0
#define PITCH_206_POS_I 0.0
#define PITCH_206_POS_D 1.0
//定义拨弹电机速度
#define SHOOT_SPEED  10   //0-10000
#endif

#ifdef CAR_HERO   //英雄车
	//定义底盘电机
	#define RM3510 

	//云台角度
	#define YAW_LEFT -850
	#define YAW_RIGHT	3150
	#define YAW_MID 1095
	#define PITCH_UP 2400
	#define PITCH_DOWN 1320
	#define PITCH_MID 1550
	//云台PID
	#define YAW_205_VEL_P 20.0
	#define YAW_205_VEL_D 0.0
	#define YAW_205_POS_P 2.0
	#define YAW_205_POS_I 0.0
	#define YAW_205_POS_D 0.1
	#define PITCH_206_VEL_P 20.0
	#define PITCH_206_VEL_D 0.0
	#define PITCH_206_POS_P 1.0
	#define PITCH_206_POS_I 0.0
	#define PITCH_206_POS_D 1.0
	
	//定义RM3510的pid
	#define ESC_820R_VEL_P 10.0//10.0
	#define ESC_820R_VEL_I 0.0
	#define ESC_820R_VEL_D 0.5//0.5
	#define ESC_820R_POS_P 10.0//10.0
	#define ESC_820R_POS_I 0.0
	#define ESC_820R_POS_D 0.0


	#define SHOOT_SPEED  10   //0-10000
#endif

#ifdef CAR_A
	//定义底盘电机
	#define RM3510 

	//云台角度
	#define YAW_LEFT 750
	#define YAW_RIGHT	4880
	#define YAW_MID 2800
	#define PITCH_UP 775
	#define PITCH_DOWN -300
	#define PITCH_MID 150
	//云台PID
	#define YAW_205_VEL_P 20.0
	#define YAW_205_VEL_D 0.0
	#define YAW_205_POS_P 2.0
	#define YAW_205_POS_I 0.0
	#define YAW_205_POS_D 0.1
	#define PITCH_206_VEL_P 20.0
	#define PITCH_206_VEL_D 0.0
	#define PITCH_206_POS_P 1.0
	#define PITCH_206_POS_I 0.0
	#define PITCH_206_POS_D 1.0
	
	//定义RM3510的pid
	#define ESC_820R_VEL_P 10.0//10.0
	#define ESC_820R_VEL_I 0.0
	#define ESC_820R_VEL_D 0.5//0.5
	#define ESC_820R_POS_P 10.0//10.0
	#define ESC_820R_POS_I 0.0
	#define ESC_820R_POS_D 0.0

	#define SHOOT_SPEED  13   //0-10000
#endif

#ifdef CAR_B
	//定义底盘电机
	#define RM3510 

	//云台角度
	#define YAW_LEFT 1480
	#define YAW_RIGHT	5630
	#define YAW_MID 3400
	#define PITCH_UP 5200
	#define PITCH_DOWN 3920
	#define PITCH_MID 4350
	//云台PID
	#define YAW_205_VEL_P 20.0
	#define YAW_205_VEL_D 0.0
	#define YAW_205_POS_P 2.0
	#define YAW_205_POS_I 0.0
	#define YAW_205_POS_D 0.1
	#define PITCH_206_VEL_P 18.0
	#define PITCH_206_VEL_D 0.0
	#define PITCH_206_POS_P 1.0
	#define PITCH_206_POS_I 0.0
	#define PITCH_206_POS_D 1.0
	
	//定义RM3510的pid
	#define ESC_820R_VEL_P 10.0//10.0
	#define ESC_820R_VEL_I 0.0
	#define ESC_820R_VEL_D 0.5//0.5
	#define ESC_820R_POS_P 10.0//10.0
	#define ESC_820R_POS_I 0.0
	#define ESC_820R_POS_D 0.0

	#define SHOOT_SPEED  10   //0-10000
#endif

#ifdef CAR_C
	//定义底盘电机
	#define RM3510 

	//云台角度
	#define YAW_LEFT 1200
	#define YAW_RIGHT	5840
	#define YAW_MID 3550
	#define PITCH_UP 4980
	#define PITCH_DOWN 3820
	#define PITCH_MID 4320
	//云台PID
	#define YAW_205_VEL_P 20.0
	#define YAW_205_VEL_D 0.0
	#define YAW_205_POS_P 2.0
	#define YAW_205_POS_I 0.0
	#define YAW_205_POS_D 0.1
	#define PITCH_206_VEL_P 20.0
	#define PITCH_206_VEL_D 0.0
	#define PITCH_206_POS_P 1.0
	#define PITCH_206_POS_I 0.0
	#define PITCH_206_POS_D 1.0
	
	//定义RM3510的pid
#define ESC_820R_VEL_P 10.0//10.0
#define ESC_820R_VEL_I 0.0
#define ESC_820R_VEL_D 0.5//0.5
#define ESC_820R_POS_P 10.0//10.0
#define ESC_820R_POS_I 0.0
#define ESC_820R_POS_D 0.0

	#define SHOOT_SPEED  5   //0-10000
#endif

//底盘运动速度定义
 #define NormalSpeed 180
 #define HighSpeed 230
 #define LowSpeed 50

//仅仅定义驱动板就可以知道用RM35电机了
#ifdef ROBOMODULE_NEW
#define RM35
#endif

#ifdef ROBOMODULE_OLD
#define RM35
#endif


//之后再整合吧
//定义摩擦轮速度
#define RUB_SPEED    1400  //1000-2200



//时序NVIC配置
#define MPU6050_Channel EXTI4_IRQn
#define MPU6050_PreemptionPriority 0
#define MPU6050_SubPriority 0

#define HMC5883L_Channel EXTI3_IRQn
#define HMC5883L_PreemptionPriority 0
#define HMC5883L_SubPriority 0

#define CAN1_RX_Channel CAN1_RX0_IRQn
#define CAN1_RX_PreemptionPriority 0
#define CAN1_RX_SubPriority 0

#define CAN1_TX_Channel CAN1_TX_IRQn
#define CAN1_TX_PreemptionPriority 1
#define CAN1_TX_SubPriority 1

#define CAN2_RX_Channel CAN2_RX1_IRQn
#define CAN2_RX_PreemptionPriority 0
#define CAN2_RX_SubPriority 2

#define CAN2_TX_Channel CAN2_TX_IRQn
#define CAN2_TX_PreemptionPriority 0
#define CAN2_TX_SubPriority 0

#define DMA1_Channel DMA1_Stream1_IRQn
#define DMA1_PreemptionPriority 0
#define DMA1_SubPriority 2

#define USART3_Channel USART3_IRQn
#define USART3_PreemptionPriority 3
#define USART3_SubPriority 3

#define DMA2_Channel DMA2_Stream5_IRQn
#define DMA2_PreemptionPriority 0
#define DMA2_SubPriority 1

#define TIM6_Channel TIM6_DAC_IRQn
#define TIM6_PreemptionPriority 3
#define TIM6_SubPriority 2


//引入库
#include "stm32f4xx.h"
//初始化
#include "initialization.h"
//DBUS解析和传输
#include "dbus.h"
#include "usart1.h"
//摩擦轮和拨弹电机
#include "pwm.h"
//CAN1云台，CAN2地盘电机
#include "can1.h"
#include "can2.h"
//用于timer6用于短时中断，timer2用于计时器
#include "timer.h"
//用于编码器配置
#include "encoder.h"
//NVIC
#include "nvic.h"
//用于调试和向ROS传送信息
#include "usart3.h"
//控制
#include "shoot_control.h"
#include "gimbal_control.h"
#include "chassis_control.h"
//PID
#include "pid_algorithm.h"
//MPU6050 HMC5883L库
#include "mpu6050_hmc5883l_driver.h"
#include "mpu6050_hmc5883l_i2c.h"
#include "mpu6050_hmc5883l_interrupt.h"
#include "mpu6050_hmc5883l_process.h"
//辅助调试和延时
#include "led.h"
#include "buzzer.h"
#include "delay.h"
//激光
#include "laser.h"
//其他基本库
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
//添加sdbus传输协议
#include <sdbus.h>

//常用函数 取最大值和绝对值
#define abs(x) ((x)>0? (x):(-(x)))
#define maxs(a,b) (a>b? a:b)

//用作串口调试printf
extern char buffer1[32];

#endif 
