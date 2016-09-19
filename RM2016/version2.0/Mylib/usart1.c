/************************************************************************************
  File Name     :  usart1.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  usart3的配置，用于DBUS接收机的接收
									 -----USART1_RX-----PB7----
									 配置gpio、usart初始化、dma及其nvic的usart1配置函数，
									 定义了dbus的对象DBUS,
									 在dma中断中进行解码
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			usart1配置函数 配置gpio、usart初始化、dma和nvic		
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  注释			   
************************************************************************************/
#include "main.h"


unsigned char dbus_buf[DBUS_BUF_SIZE];

void USART1_Config(void)
{
    USART_InitTypeDef usart1;
    GPIO_InitTypeDef  gpio;
    DMA_InitTypeDef   dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_USART1);
    
    gpio.GPIO_Pin = GPIO_Pin_7 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);
    
    USART_DeInit(USART1);
    usart1.USART_BaudRate = 100000;   //SBUS 100K baudrate
    usart1.USART_WordLength = USART_WordLength_8b;
    usart1.USART_StopBits = USART_StopBits_1;
    usart1.USART_Parity = USART_Parity_Even;
    usart1.USART_Mode = USART_Mode_Rx;
    usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&usart1);
    
    USART_Cmd(USART1,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    
		NVIC_Set(DMA2_Channel,DMA2_PreemptionPriority,DMA2_SubPriority,ENABLE);
    
    DMA_DeInit(DMA2_Stream5);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (u32)&(USART1->DR);
    dma.DMA_Memory0BaseAddr = (u32)dbus_buf;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = DBUS_BUF_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_Mode_Normal;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5,&dma);

    DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA2_Stream5,ENABLE);
}
DBUS dbus;
void DMA2_Stream5_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
    {
       
        DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);

        /*******************decode DBUS data*******************/
        DBUS_Dec(&dbus,dbus_buf);
				
				
				
				SDState_Set(dbus.key.v);
				
				if(SDFlag==0)
				{
					Trigger_Control(dbus.mouse.x,dbus.mouse.y,dbus.rc.ch3);
				}
				if(SDFlag==1)
				{
					if (dbus.mouse.y>3) {Pitch.targetAngle += -15;}
					if (dbus.mouse.y<-3) {Pitch.targetAngle += 15;}
					Pitch.targetAngle += (dbus.rc.ch3-1024)/33;
				}
		}
}
