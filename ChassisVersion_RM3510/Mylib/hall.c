#include "main.h"

void Hall_Configuration(void)
{
    GPIO_InitTypeDef  gpio;
    EXTI_InitTypeDef  exti;
    NVIC_InitTypeDef  nvic;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,  ENABLE);
    
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &gpio);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource11);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource12);
    
    exti.EXTI_Line = EXTI_Line10 | EXTI_Line11 | EXTI_Line12;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升沿下降沿都检测
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
    nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

int direction = 1;          //正转为1，反转为-1
unsigned int BLDC_PWM=0;  //电机转动占空比，满值为1000，但到不了100%

void EXTI15_10_IRQHandler(void)
{    
    unsigned char hall_state = 0;   //霍尔状态
    
    hall_state = (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)<<2)|
                 (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)<<1)|
                 (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12));
//    
//    //MAXON EC60
//    if(direction == 1)
//    {
//        switch(hall_state)
//        {
//            case 3:{  H1_PWM = 0;         H2_PWM = 0;         H3_PWM = BLDC_PWM;  L1_ON();  L2_OFF();  L3_OFF(); }break;          
//            case 2:{  H1_PWM = 0;         H2_PWM = 0;         H3_PWM = BLDC_PWM;  L1_OFF(); L2_ON();   L3_OFF(); }break;    
//            case 6:{  H1_PWM = BLDC_PWM;  H2_PWM = 0;         H3_PWM = 0;         L1_OFF(); L2_ON();   L3_OFF(); }break;
//            case 4:{  H1_PWM = BLDC_PWM;  H2_PWM = 0;         H3_PWM = 0;         L1_OFF(); L2_OFF();  L3_ON();  }break;
//            case 5:{  H1_PWM = 0;         H2_PWM = BLDC_PWM;  H3_PWM = 0;         L1_OFF(); L2_OFF();  L3_ON();  }break;         
//            case 1:{  H1_PWM = 0;         H2_PWM = BLDC_PWM;  H3_PWM = 0;         L1_ON();  L2_OFF();  L3_OFF(); }break;         
//            default:break;
//        }
//    }
//    if(direction == -1)
//    {
//        switch(hall_state)
//        {     
//            case 3:{  H1_PWM = BLDC_PWM;  H2_PWM = 0;         H3_PWM = 0;         L1_OFF(); L2_OFF();  L3_ON();  }break;
//            case 2:{  H1_PWM = 0;         H2_PWM = BLDC_PWM;  H3_PWM = 0;         L1_OFF(); L2_OFF();  L3_ON();  }break;         
//            case 6:{  H1_PWM = 0;         H2_PWM = BLDC_PWM;  H3_PWM = 0;         L1_ON();  L2_OFF();  L3_OFF(); }break;   
//            case 4:{  H1_PWM = 0;         H2_PWM = 0;         H3_PWM = BLDC_PWM;  L1_ON();  L2_OFF();  L3_OFF(); }break;    
//            case 5:{  H1_PWM = 0;         H2_PWM = 0;         H3_PWM = BLDC_PWM;  L1_OFF(); L2_ON();   L3_OFF(); }break;    
//            case 1:{  H1_PWM = BLDC_PWM;  H2_PWM = 0;         H3_PWM = 0;         L1_OFF(); L2_ON();   L3_OFF(); }break;            
//            default:break;
//        }
//    }
    
      //MAXON EC90 
//    if(direction == 1)
//    {
//        switch(hall_state)
//        {
//            case 3:{  H1_PWM = 0;         H2_PWM = BLDC_PWM;  H3_PWM = 0;          L1_ON();  L2_OFF(); L3_OFF(); }break;
//            case 2:{  H1_PWM = 0;         H2_PWM = BLDC_PWM;  H3_PWM = 0;          L1_OFF(); L2_OFF(); L3_ON();  }break;
//            case 6:{  H1_PWM = BLDC_PWM;  H2_PWM = 0;         H3_PWM = 0;          L1_OFF(); L2_OFF(); L3_ON();  }break;  
//            case 4:{  H1_PWM = BLDC_PWM;  H2_PWM = 0;         H3_PWM = 0;          L1_OFF(); L2_ON();  L3_OFF(); }break;   
//            case 5:{  H1_PWM = 0;         H2_PWM = 0;         H3_PWM = BLDC_PWM;   L1_OFF(); L2_ON();  L3_OFF(); }break;  
//            case 1:{  H1_PWM = 0;         H2_PWM = 0;         H3_PWM = BLDC_PWM;   L1_ON();  L2_OFF(); L3_OFF(); }break;
//            default:break;
//        }
//    }
//    if(direction == -1)
//    {
//        switch(hall_state)
//        {   
//            case 3:{  H1_PWM = BLDC_PWM;  H2_PWM = 0;         H3_PWM = 0;          L1_OFF(); L2_ON();  L3_OFF(); }break;     
//            case 2:{  H1_PWM = 0;         H2_PWM = 0;         H3_PWM = BLDC_PWM;   L1_OFF(); L2_ON();  L3_OFF(); }break;  
//            case 6:{  H1_PWM = 0;         H2_PWM = 0;         H3_PWM = BLDC_PWM;   L1_ON();  L2_OFF(); L3_OFF(); }break;
//            case 4:{  H1_PWM = 0;         H2_PWM = BLDC_PWM;  H3_PWM = 0;          L1_ON();  L2_OFF(); L3_OFF(); }break;
//            case 5:{  H1_PWM = 0;         H2_PWM = BLDC_PWM;  H3_PWM = 0;          L1_OFF(); L2_OFF(); L3_ON();  }break;
//            case 1:{  H1_PWM = BLDC_PWM;  H2_PWM = 0;         H3_PWM = 0;          L1_OFF(); L2_OFF(); L3_ON();  }break;  
//   
//            default:break;
//        }
//    }    
    
    printf("%d",hall_state);
    
    if(EXTI_GetITStatus(EXTI_Line10))
    {
        EXTI_ClearITPendingBit(EXTI_Line10);
    }

    if(EXTI_GetITStatus(EXTI_Line11))
    {
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
    
    if(EXTI_GetITStatus(EXTI_Line12))
    {
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
