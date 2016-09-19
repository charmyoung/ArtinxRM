/************************************************************************************
  File Name     :  initialization.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  main中各个模块的初始化，注意timerstart和控制相应部分初始化的位置
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			   初始化	
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  注释	
************************************************************************************/
#include "main.h"

void  Initialization(void)
{
	
		int i = 0;//用于初始化mpu6050和hmc5883l
		SystemInit();//系统时钟配置
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC优先级组2
		
		Led_Configuration();//LED初始化，用于mpu6050和hmc5883l的初始化和Debug
		Laser_Configuration();//Laser初始化
		USART1_Config();//串口1的配置，DBUS RX；
		USART3_Configuration();//串口3的配置，用于调试；
		
		TIM2_Configuration();//用于计时
	  TIM6_Configuration();//云台控制中断
		CAN1_Configuration();//	Can1的配置，用于控制云台
		CAN2_Configuration();//Can2的配置，用于控制底盘	
	  Quad_Encoder_Configuration();//配置拨弹电机编码器
	  Encoder_Start();

		delay_ms(500);    
		
		while(MPU6050_Initialization() == 0xff || HMC5883L_Initialization() == 0xff) 
    {
        i++;     //如果一次初始化没有成功，那就再来一次                     
        if(i>10) //如果初始化一直不成功，那就没希望了，进入死循环，绿色LED一直闪
        {
            while(1) 
            {
                LED_GREEN_TOGGLE();
                delay_ms(50);
                
            }
        }  
    } 
		
    MPU6050_Gyro_calibration();//MPU6050校准
    MPU6050_HMC5883L_Interrupt_Configuration(); //HMC和MPU的中断配置
		
		PWM_Configuration();//PWM配置，用于控制摩擦轮和拨弹电机
		
		TIM6_Start(); 
		delay_ms(500);
		
		#ifdef RM35
		CAN_RoboModule_DRV_Reset(0,0);//CAN重置
		delay_ms(1000);
		CAN_RoboModule_DRV_Mode_Choice(0,0,PWM_VELOCITY_MODE);//PWM速度模式
		delay_ms(1000);
		#endif
		

}
