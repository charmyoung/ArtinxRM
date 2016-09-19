/************************************************************************************
  File Name     :  laser.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  用于配置激光及其开启，
									 ----LASER-----PA8----
									 laser.h中定义了相关函数LASER_ON()和LASER_OFF()

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			  laser配置
2     1.2       6/29       gyf 					
3     1.3       6/29       yf 					  注释			   
************************************************************************************/
#include "main.h"

void Laser_Configuration()
{
	GPIO_InitTypeDef gpioInitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_8;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

