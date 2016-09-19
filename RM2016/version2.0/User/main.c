/************************************************************************************
  File Name     :  main.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :   main函数主要用于程序接口，其中包含初始化部分和地盘控制及发射部分；
										main.h头文件主要包括了几乎所有库的引用及战车各种参数的配置，包括了
										云台的角度参数，云台整定PID参数，底盘电机种类，摩擦轮电机速度以及
										NVIC的配置，还有一些常用函数的定义。
										
  
-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   					main函数	 包含初始化部分和地盘控制及发射部分
2     1.2       6/29       gyf 
3     1.3       6/29       yf 						注释						
************************************************************************************/	
#include "main.h"

char buffer1[32];//用于串口printf，主要调试用；


int main(void)
{   
		
		Initialization();
	
		while(1)
		{	
		
				if(DBUS_Det(dbus))//rc开启判断
			{
				#ifdef RM35
				move_control(dbus.rc.ch0, dbus.rc.ch1, dbus.rc.ch2, dbus.rc.s1, dbus.key.v);
				#endif
				
				BLDC_control(dbus.rc.s2, dbus.mouse.r);
				Fire(dbus.rc.s2,dbus.mouse.l);				
				
			}
			//串口测试例子
			//delay_ms(50);

		//	delay_ms(20);
	//	int	g=GetQuadEncoderDiff();
			//	ShootMotorSpeedSet((int)Velocity_Control_Shoot(g,10));
		//	sprintf(buffer1,"%d %d\n",g,PWM3);
			//sprintf(buffer1,"%d %d\n",Yaw.thisAngle,Pitch.thisAngle);
		//	printf(buffer1);
			
		}
}
