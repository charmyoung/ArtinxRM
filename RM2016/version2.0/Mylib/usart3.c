/************************************************************************************
  File Name     :  usart3.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  usart3的配置，用于调试发送用，以及后期和ros通信部分
									 -----USART3_TX-----PB10-----
								 	 -----USART3_RX-----PB11-----
									 配置gpio、usart初始化和nvic的usart3配置函数，
									 用于调试的发送char，int，string函数
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			usart3配置函数	配置gpio、usart初始化和nvic		
																				发送函数			发送char，int，string
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  注释		
4     1.6       7/8        
************************************************************************************/
#include "main.h"
unsigned char USART_RX_BUF[USART_REC_LEN];

void USART3_Configuration(void)
{
    USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
	   DMA_InitTypeDef   dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
	
		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB,&gpio);

		usart3.USART_BaudRate =  9600;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

    //USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
		USART_Cmd(USART3,ENABLE);
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		
		NVIC_Set(USART3_Channel,USART3_PreemptionPriority,USART3_SubPriority,ENABLE);
		NVIC_Set(DMA1_Channel,DMA1_PreemptionPriority,DMA1_SubPriority,ENABLE);
		
		DMA_DeInit(DMA1_Stream1);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)USART_RX_BUF;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = USART_REC_LEN;
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
    DMA_Init(DMA1_Stream1,&dma);
		#ifdef AUTO_TRACK
    DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
    #endif
		DMA_Cmd(DMA1_Stream1,ENABLE);
}

void DMA1_Stream1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
    {
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);

				SDBUS_Dec(&sdbus,USART_RX_BUF);	

				if(SDFlag==1)
				{
					SD_TriggerControl();
					//printf("%d %d\n",sdbus.PitchAngle,sdbus.YawAngle);
				}
    }
}

void USART3_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
		USART_SendData(USART3,b);
}

void USART3_SendStr(uint8_t *str)
{
	while(0 != *str){
	USART3_SendChar(*str);
	str++;
	}
}
int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3, (uint8_t)ch);
    return ch;
}

//bit15,	0x0a
//bit14,	0x0d
//bit13~0
/*
u16 USART_RX_STA=0;



void USART3_IRQHandler(void)
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		Res =USART_ReceiveData(USART3);//(USART3->DR);
		
		if((USART_RX_STA&0x8000)==0)
		{
			if(USART_RX_STA&0x4000)
			{
				if(Res!=0x0a)USART_RX_STA=0;
				else USART_RX_STA|=0x8000;	 
			}
			else 
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//溢出
				}		 
			}
		}   		 
  } 
  
}
*/
//待补充向manifold发送串口指令函数；
void USART3_TX_ROS(void)
{
	if(BlueFlag==0)
	{
		printf("red\n");
	}
	else if(BlueFlag==1)
	{
		printf("blue\n");
	}
}
