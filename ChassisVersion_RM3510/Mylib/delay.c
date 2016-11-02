/************************************************************************************
  File Name     :  delay.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  延时函数的定义，包括ms和us。由时钟决定。
									 
-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			 延时函数	     循环近似延时
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					 注释			   
************************************************************************************/
#include "main.h"

void delay_ms(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=42000;
		while(a--);
	}
}

void delay_us(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=40;
		while(a--);
	}
}
