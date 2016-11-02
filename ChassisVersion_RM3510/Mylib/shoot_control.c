/************************************************************************************
  File Name     :  shoot_control.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  用于遥控器和键鼠的双级射击控制，一级转动摩擦轮，rub_flag置位，二级转动拨弹电机，
									 shoot_flag置位。键鼠的优先级更高一些。
										
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			  两级发射控制	
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  注释	
4     1.4       7/1        yf    ShootMotor_Velocity_Control  拨弹电机速度环控制
************************************************************************************/
#include "main.h"

uint8_t last_s2;//s2的上一次key
uint8_t last_pressr;
int rub_flag;//s2-1位开启状态
int shoot_flag;//s2-2位开启状态


//拨弹电机速度环控制函数：ShootMotor_Velocity_Control(float TargetShootSpeed)在哪个timer里调用需要考虑，涉及到调节pid的值
void ShootMotor_Velocity_Control(float TargetShootSpeed)
{
	 s32 PWM_Output;
	 PWM_Output = (s32)Velocity_Control_Shoot((float)(GetQuadEncoderDiff()) ,TargetShootSpeed);
	 ShootMotorSpeedSet(PWM_Output);
}

//rc和key控制摩擦轮BLDC开启
void BLDC_control(uint8_t s2, uint8_t press_r)
{	
	//键盘键位解析
	
	//int key_G = KEY_PRESSED_OFFSET_SHIFT & v; if (key_G!=0) key_G=1;

	if (rub_flag == 1 && ( (s2==1 && last_s2!=s2) || (press_r == 1 && last_pressr!=press_r) ))
  {
    //pwm控制电调2312 close；
			PWM1=1000;
			PWM2=1000;
	  	rub_flag=0;
		  LASER_OFF();
  }
  else if (rub_flag == 0 && ( (s2==1 && last_s2!=s2) || (press_r == 1 && last_pressr!=press_r) ))
  {
    
		//pwm控制电调2312 open；
    		PWM1=RUB_SPEED;
				PWM2=RUB_SPEED;
    		rub_flag=1;
		    LASER_ON();
	}
	last_s2=s2;
	last_pressr=press_r;
}


//rc和mouse控制拨弹电机开启（开火）
void Fire(uint8_t s2, uint8_t press_l)
{
	shoot_flag=0;
	if ((rub_flag == 1) && ( (s2==2) || (press_l == 1) ))
  {

    shoot_flag=1;
		
	}
  if (shoot_flag == 1)
	{ 
		int	g=GetQuadEncoderDiff();
		ShootMotorSpeedSet((int)Velocity_Control_Shoot(g,SHOOT_SPEED));
		//printf("%d %d\n",g,PWM3);
	}
	else if(shoot_flag == 0)
	{ PWM3=0;}
	delay_ms(20);
	last_s2=s2;
}
