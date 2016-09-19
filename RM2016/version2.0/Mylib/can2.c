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
		//旧版本ROBOMODULE中可能存在can芯片不兼容的问题，所以开始不断的接收包。关闭接收中断。
		#ifdef ROBOMODULE_OLD
			CAN_ITConfig(CAN2,CAN_IT_FMP1,DISABLE);
		#endif
		
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
						
					  //RM35_ROBOMODULE_NEW
						#ifdef ROBOMODULE_NEW
            if(rx_message.StdId == 0x1B)
            {
                RM35_1.thisCurrent  = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM35_1.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                RM35_1.thisPosition = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            }
            else if(rx_message.StdId == 0x2B)
            {
                RM35_2.thisCurrent  = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM35_2.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                RM35_2.thisPosition = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            }
            else if(rx_message.StdId == 0x3B)
            {
                RM35_3.thisCurrent  = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM35_3.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                RM35_3.thisPosition = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            }
            else if(rx_message.StdId == 0x4B)
            {
                RM35_4.thisCurrent  = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM35_4.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                RM35_4.thisPosition = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
            }
            else if(rx_message.StdId == 0x1F)
            {
                RM35_1.Online = 1;
            }
            else if(rx_message.StdId == 0x2F)
            {
                RM35_2.Online = 1;
            }
            else if(rx_message.StdId == 0x3F)
            {
                RM35_3.Online = 1;
            }
            else if(rx_message.StdId == 0x4F)
            {
								RM35_4.Online = 1;
            }
            else if(rx_message.StdId == 0x1C)
            {
               RM35_1.Ctl1_Value= rx_message.Data[0];
               RM35_1.Ctl2_Value= rx_message.Data[1];
            }
            else if(rx_message.StdId == 0x2C)
            {
               RM35_2.Ctl1_Value= rx_message.Data[0];
               RM35_2.Ctl2_Value= rx_message.Data[1];
            }
            else if(rx_message.StdId == 0x3C)
            {
               RM35_3.Ctl1_Value= rx_message.Data[0];
               RM35_3.Ctl2_Value= rx_message.Data[1];
            }
            else if(rx_message.StdId == 0x4C)
            {
               RM35_4.Ctl1_Value= rx_message.Data[0];
               RM35_4.Ctl2_Value= rx_message.Data[1];
            }
						#endif
						
						
						//RM3510
						#ifdef RM3510
						if(rx_message.StdId == 0x201)
            {
                RM3510_1.thisPosition = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM3510_1.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);	
						}	
						if(rx_message.StdId == 0x202)
            {
                RM3510_2.thisPosition = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM3510_2.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);	
						}	
				    if(rx_message.StdId == 0x203)
            {
                RM3510_3.thisPosition = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM3510_3.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);	
						}	
						if(rx_message.StdId == 0x204)
            {
                RM3510_4.thisPosition = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                RM3510_4.thisVelocity = (rx_message.Data[2]<<8)|(rx_message.Data[3]);	
						}	
						#endif

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
//RM35电机——ROBOMODULE发送函数
/****************************************************************************************
                                       复位指令
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                     模式选择指令
mode的取值范围如下：
PWM_MODE
PWM_CURRENT_MODE
PWM_VELOCITY_MODE
PWM_POSITION_MODE
PWM_VELOCITY_POSITION_MODE
CURRENT_VELOCITY_MODE
CURRENT_POSITION_MODE
CURRENT_VELOCITY_POSITION_MODE
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = Mode;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}


/****************************************************************************************
                                  PWM模式下的数据指令
temp_pwm的取值范围如下：
-5000 ~ +5000
*****************************************************************************************/
void CAN_RoboModule_DRV_PWM_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                  PWM电流模式下的数据指令
temp_pwm的取值范围如下：
0 ~ +5000

temp_current的取值范围如下：
-1600 ~ +1600
*****************************************************************************************/
void CAN_RoboModule_DRV_PWM_Current_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Current > 2000)
    {
        Temp_Current = 2000;
    }
    else if(Temp_Current < -2000)
    {
        Temp_Current = -2000;
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                  PWM速度模式下的数据指令
temp_pwm的取值范围如下：
0 ~ +5000

temp_velocity的取值范围如下：
-32768 ~ +32767
*****************************************************************************************/
void CAN_RoboModule_DRV_PWM_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);

}

/****************************************************************************************
                                  PWM位置模式下的数据指令
temp_pwm的取值范围如下：
0 ~ +5000

temp_position的取值范围如下：
32位有符号整型
*****************************************************************************************/
void CAN_RoboModule_DRV_PWM_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                  PWM速度位置模式下的数据指令
temp_pwm的取值范围如下：
0 ~ +5000

temp_velocity的取值范围如下：
0 ~ +32767

temp_position的取值范围如下：
32位有符号整型
*****************************************************************************************/
void CAN_RoboModule_DRV_PWM_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                  电流速度模式下的数据指令
temp_current的取值范围如下：
0 ~ +2000

temp_velocity的取值范围如下：
-32768 ~ +32767
*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_Current > 2000)
    {
        Temp_Current = 2000;
    }
    else if(Temp_Current < -2000)
    {
        Temp_Current = -2000;
    }
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                  电流位置模式下的数据指令
temp_current的取值范围如下：
0 ~ +2000

temp_position的取值范围如下：
32位有符号整型
*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_Current > 2000)
    {
        Temp_Current = 2000;
    }
    else if(Temp_Current < -2000)
    {
        Temp_Current = -2000;
    }
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }

    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                  电流速度位置模式下的数据指令
temp_current的取值范围如下：
0 ~ +2000

temp_velocity的取值范围如下：
0 ~ +32767

temp_position的取值范围如下：
32位有符号整型
*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_Current > 2000)
    {
        Temp_Current = 2000;
    }
    else if(Temp_Current < -2000)
    {
        Temp_Current = -2000;
    }
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }

    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                      配置指令
Temp_Time的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Ctl1_Ctl2的取值范围：0 or 1 ，当不为0 or 1，则认为是0，为关闭左右限位检测功能
特别提示：Ctl1，Ctl2的功能仅存在于RMDS-102，其余版本驱动器，Ctl1_Ctl2 = 0 即可
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2)
{
    unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x1: //本指令不支持广播，所以此处没有0号驱动器
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    if((Ctl1_Ctl2 != 0x00)&&(Ctl1_Ctl2 != 0x01))
    {
        Ctl1_Ctl2 = 0x00;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = Temp_Time;
    tx_message.Data[1] = Ctl1_Ctl2;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}

/****************************************************************************************
                                      在线检测
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    switch(Group)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7: can_id |= Group<<8; break;
        default: return;
    }
    
    switch(Number)
    {
        case 0x1: //本指令不支持广播，所以此处没有0号驱动器
        case 0x2:
        case 0x3:
        case 0x4:
        case 0x5:
        case 0x6:
        case 0x7:
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xC:
        case 0xD:
        case 0xE:
        case 0xF: can_id |= Number<<4; break;
        default: return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can2_tx_success_flag = 0;
    CAN_Transmit(CAN2,&tx_message);
    while(can2_tx_success_flag == 0);
}
