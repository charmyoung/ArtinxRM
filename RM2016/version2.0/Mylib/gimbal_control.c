/************************************************************************************
  File Name     :  gimbal_control.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  针对6623云台pithc和yaw轴运动的自下而上的控制。
									 其中首先是两轴云台（2DOF）的PID位置环的控制，
									 然后是云台远程控制函数（遥控器和键鼠），键鼠优先级更高一些。
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			  两层云台控制	
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  注释			   
************************************************************************************/
#include "main.h"

//初始化云台角度
M6623 Yaw = {YAW_LEFT,YAW_RIGHT,YAW_MID,0,0,0,0,YAW_MID,0,0};
M6623 Pitch = {PITCH_DOWN,PITCH_UP,PITCH_MID,0,0,0,0,PITCH_MID,0,0};

/*********************************************************************
Name：          void Gimbal_Control(void)  

Description：  云台控制程序
               向上运动电流为正值
*********************************************************************/
void Gimbal_Control(void)  
{
	//外环PID控制
	//计算位置闭环输出量
	Yaw.position_output = Position_Control_205(Yaw.thisAngle,Yaw.targetAngle);
	//内环PID控制
  Yaw.velocity_output = Velocity_Control_205(-MPU6050_Real_Data.Gyro_Z ,Yaw.position_output);

	//计算位置闭环输出量
	Pitch.position_output = Position_Control_206(Pitch.thisAngle,Pitch.targetAngle);
	//内环PID控制
  Pitch.velocity_output = Velocity_Control_206(-MPU6050_Real_Data.Gyro_Y ,Pitch.position_output);
	Cmd_ESC(Yaw.velocity_output,Pitch.velocity_output);
	
}

/*********************************************************************
Name：         void Trigger_Control(int16_t x, int16_t y, uint16_t ch3)

Description：  云台远程控制程序（遥控器和键盘）            
*********************************************************************/

void Trigger_Control(int16_t x, int16_t y, uint16_t ch3)
{
				//暂时不用yaw轴
				if (Yaw.targetAngle < Yaw.minAngle){Yaw.targetAngle=Yaw.minAngle;}
				if (Yaw.targetAngle > Yaw.maxAngle){Yaw.targetAngle=Yaw.maxAngle;}

			
				if (y>3) {Pitch.targetAngle += -15;}
        if (y<-3) {Pitch.targetAngle += 15;}
				Pitch.targetAngle += (ch3-1024)/33;
			
				if (Pitch.targetAngle < Pitch.minAngle){Pitch.targetAngle=Pitch.minAngle;}
				if (Pitch.targetAngle > Pitch.maxAngle){Pitch.targetAngle=Pitch.maxAngle;}		
			
}
