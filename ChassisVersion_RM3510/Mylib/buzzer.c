#include "main.h"
/************************************************************************************
  File Name     :  buzzer.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  未使用！！！
									----Buzzer----PA5-----'1' is on,'0' is off
									 BUZZER的配置主要是用于调试
									 buzzer.h中主要定义了快捷改变LED的函数 如BUZZER_ON()、
									 BUZZER_OFF()、BUZZER_TOGGLE()函数
-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			 BUZZER配置函数	 配置BUZZER的GPIO及初始化
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					 注释			   	该部分未使用
************************************************************************************/	

void Buzzer_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
    
    BUZZER_OFF();
}
