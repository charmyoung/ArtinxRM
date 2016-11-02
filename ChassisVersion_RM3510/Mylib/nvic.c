/************************************************************************************
  File Name     :  nvic.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  NVIC_Set用于快捷配置各部分的优先级，主要注意channel，抢占优先级和
									 子优先级的配置；
									 NVIC_Configuration用于所有需要配置nvic的模块的整体配置。
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			   	null
2     1.2       6/29       gyf 					增加了nvic.c  增加NVIC_Set函数
3     1.3       6/29       yf 					注释及nvic.h	将nvic中各部分的配置移动到main.h中	   
************************************************************************************/
#include "nvic.h"
void NVIC_Configuration(void){
  
}

void NVIC_Set(int Channel,int PreemptionPriority,int SubPriority,FunctionalState Cmd){
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = Channel;
	nvic.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
	nvic.NVIC_IRQChannelSubPriority = SubPriority;
	nvic.NVIC_IRQChannelCmd = Cmd;
	NVIC_Init(&nvic);
}
