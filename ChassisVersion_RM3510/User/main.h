#ifndef __MAIN_H__
#define __MAIN_H__



//定义工作与调试模式
//#define DEBUG


//定义RM3510的pid
#define ESC_820R_VEL_P 150.0//10.0
#define ESC_820R_VEL_I 0.0
#define ESC_820R_VEL_D 5//0.5
#define ESC_820R_POS_P 10.0//10.0
#define ESC_820R_POS_I 0.0
#define ESC_820R_POS_D 0.0


//底盘运动速度定义
 #define NormalSpeed 10
 #define HighSpeed 15
 #define LowSpeed 5


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

//CAN1云台，CAN2地盘电机
#include "can1.h"
#include "can2.h"
//timer2用于计时器
#include "timer.h"

//NVIC
#include "nvic.h"
//用于调试和向ROS传送信息
#include "usart3.h"
//控制
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
