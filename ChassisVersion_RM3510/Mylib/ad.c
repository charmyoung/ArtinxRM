#include "main.h"

//ADC123_IN2(PA2)

void Power_Detection_Configuration(void)
{
    ADC_InitTypeDef adc;
    GPIO_InitTypeDef gpio;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&gpio);
    
    adc.ADC_Resolution = ADC_Resolution_10b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1,&adc);
    
    ADC_Cmd(ADC1,ENABLE);
    
    ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_56Cycles);
}

//µÁ‘¥µÁ—πºÏ≤‚
void Power_Detect(void)
{
    unsigned int AD_Value;

    ADC_SoftwareStartConv(ADC1);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==RESET);

    AD_Value = ADC_GetConversionValue(ADC1);
    if(AD_Value < 3000)
    {
        //....
    }
}
