/************************************************************************************
  File Name     :  can2.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  can1的配置，用于控制底盘电机与板间通信。
									 ----CAN2_TX-----PB13----
									 ----CAN2_RX-----PB12----
									 配置GPIO和CAN初始化和CANFilter的can2配置函数，发送中断，接收中断
									 （接收两类电机驱动板数据），给RM3510电调发送电流环函数，给RM35驱动板
									 发送命令的多个函数

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			 	 	can2				
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  注释			   
************************************************************************************/
#include "main.h"

void CAN2_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    NVIC_Set(CAN2_RX_Channel,CAN2_RX_PreemptionPriority,CAN2_RX_SubPriority,ENABLE); 
		NVIC_Set(CAN2_TX_Channel,CAN2_TX_PreemptionPriority,CAN2_TX_SubPriority,ENABLE); 
		
    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;    
    can.CAN_AWUM = DISABLE;    
    can.CAN_NART = DISABLE;    
    can.CAN_RFLM = DISABLE;    
    can.CAN_TXFP = ENABLE;     
    can.CAN_Mode = CAN_Mode_Normal; 
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=1;//the message which pass the filter save in fifo1
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
		
		CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
		CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);

		
}

unsigned char can2_tx_success_flag = 0;
/*************************************************************************
                          CAN2_TX_IRQHandler
描述：CAN2的发送中断函数
*************************************************************************/
void CAN2_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
       can2_tx_success_flag=1;
    }
}



//本接收数据的函数，默认为4个驱动器，都挂在0组，编号为1、2、3、4
/*************************************************************************
                          CAN2_RX1_IRQHandler
描述：CAN2的接收中断函数
*************************************************************************/
void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP1)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx_message);
			
			  if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.RTR == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
        {		
						
						
						//RM3510
						if(rx_message.StdId == 0x201)
            {		
								RM3510_1.lastAngle = RM3510_1.thisAngle; 
                RM3510_1.thisAngle = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM3510_1.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);	
								RM3510_1.thisPosition += (double)GetAngleDiff(RM3510_1.lastAngle,RM3510_1.thisAngle)/8191.0;
								
						}	
						if(rx_message.StdId == 0x202)
            {
								RM3510_2.lastAngle = RM3510_2.thisAngle;
                RM3510_2.thisAngle = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM3510_2.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);	
								RM3510_2.thisPosition += (double)GetAngleDiff(RM3510_2.lastAngle,RM3510_2.thisAngle)/8191.0;
						}	
				    if(rx_message.StdId == 0x203)
            {
								RM3510_3.lastAngle = RM3510_3.thisAngle;
                RM3510_3.thisAngle = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM3510_3.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);	
								RM3510_3.thisPosition += (double)GetAngleDiff(RM3510_3.lastAngle,RM3510_3.thisAngle)/8191.0;
						}	
						if(rx_message.StdId == 0x204)
            {
								RM3510_4.lastAngle = RM3510_4.thisAngle;
                RM3510_4.thisAngle = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM3510_4.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);	
								RM3510_4.thisPosition += (double)GetAngleDiff(RM3510_4.lastAngle,RM3510_4.thisAngle)/8191.0;
						}	
						
        }
		}
}
//RM3510电机
void Cmd_ESC_820R(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(current_201 >> 8);
    tx_message.Data[1] = (unsigned char)current_201;
    tx_message.Data[2] = (unsigned char)(current_202 >> 8);
    tx_message.Data[3] = (unsigned char)current_202;
    tx_message.Data[4] = (unsigned char)(current_203 >> 8);
		tx_message.Data[5] = (unsigned char)current_203;
    tx_message.Data[6] = (unsigned char)(current_204 >> 8);
    tx_message.Data[7] = (unsigned char)current_204;
    
    //can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    //while(can2_tx_success_flag == 0);
}
//得到角度差
int GetAngleDiff(int16_t lastAngle, int16_t thisAngle)
{
	if(lastAngle-thisAngle<-7000){
		return thisAngle-lastAngle-8191;	
	}
	else if(lastAngle-thisAngle>7000){
		return thisAngle-lastAngle+8191;
	}
	else
		return thisAngle-lastAngle;
}
