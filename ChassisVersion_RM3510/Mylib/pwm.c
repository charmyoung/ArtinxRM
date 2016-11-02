/************************************************************************************
  File Name     :  pwm.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  pwm的配置，对应关系如下。占用TIME5和TIME9。
										PWM3---(PA2---TIM9_CH1)
										PWM2---(PA1---TIM5_CH2)
										PWM1---(PA0---TIM5_CH1)
									 pwm.h 快捷定义了PWM对应CCR的关系。
-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			 PWM配置函数	 配置GPIO、TIM初始化及OC，CCR初始化
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  注释			   
4			1.4       7/1				 yf        ShootMotorSpeedSet   控制pwm3即拨弹电机速度
************************************************************************************/
#include "main.h"

//测试完毕后再放进头文件中，需要调试的是pwmperiod的取值和prescaler的值，pwmloss是仿照其他的程序，可微调
int PWMPeriod =20000;
int PWMLOSS=2000;

void PWM_Configuration(void)
{  
    
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   //PCLK1=42MHz,TIM5 clk =84MHz
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);   //PCLK1=42MHz,TIM9 clk =84MHz

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); 
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9);	
    /* TIM5 */
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&tim);
		
    /* TIM9 */
		tim.TIM_Prescaler = 168-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = PWMPeriod;   //1ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9,&tim);
		
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM5,&oc);
    TIM_OC2Init(TIM5,&oc);
		oc.TIM_Pulse = 0;
		TIM_OC1Init(TIM9,&oc);
    
    TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
		TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    TIM_ARRPreloadConfig(TIM9,ENABLE);
		
    TIM_Cmd(TIM5,ENABLE);
		TIM_Cmd(TIM9,ENABLE);
		
		PWM1 = 1000;//1、2摩擦轮电机
		PWM2 = 1000;
		PWM3= 0;//拨弹电机

}
void ShootMotorSpeedSet(s32 MotorSpeed)//拨弹电机设置
{		
	if(MotorSpeed > PWMPeriod-PWMLOSS)    MotorSpeed =   PWMPeriod-PWMLOSS ;//上限 CCR的值必须小于或等于ARR的值
	PWM3 = MotorSpeed;
}

