/************************************************************************************
  File Name     :  mpu6050_hmc5883l_interrupt.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  配置mpu6050和hmc5883l的接收引脚GPIO和其外部中断
										----mpu6050-----PA4-----EXTI4----
										----hmc5883-----PA3-----EXTI3----
									 
-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       								Description   
1     1.1       6/28       yf   		EXTI4_IRQHandler      	在mpu6050的中断中跑6050数据接收及rm3510电机的底盘控制和云台控制
																		EXTI3_IRQHandler				在hmc5883l的中断中跑地磁数据接收和ahrs算法确定实时q和欧拉角
2     1.2       6/29       gyf 			EXTI4_IRQHandler        在mpu6050中断中只跑6050数据接收及rm3510电机的底盘控制，云台控制移动到timer6中跑         
3     1.3       6/29       yf 					  注释			   
************************************************************************************/
#include "main.h"
uint32_t a=0;
void MPU6050_HMC5883L_Interrupt_Configuration(void)
{
    GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;
 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);   
 
	  gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &gpio);
	
		gpio.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOA, &gpio);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource3); 
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource4); 
		
    exti.EXTI_Line = EXTI_Line4;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿中断
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
		
		exti.EXTI_Line = EXTI_Line3;
    EXTI_Init(&exti);
		
		NVIC_Set(MPU6050_Channel,MPU6050_PreemptionPriority,MPU6050_SubPriority,ENABLE);
	//	NVIC_Set(HMC5883L_Channel,HMC5883L_PreemptionPriority,HMC5883L_SubPriority,ENABLE);

}

//MPU6050 外部中断处理函数
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4) == SET)
    {
        //2ms进中断一次
        //读取MPU6050数据,为了使云台的控制更平滑，
        //使用MPU6050的陀螺仪输出作为速度环反馈
        //单纯使用电调板返回机械角度值做速度环反馈，会有明显振荡现象
        
				MPU6050_ReadData();//读取未滤波数据                                              
        MPU6050_Data_Filter();//主要是mpu6050加速度计的均值滤波和陀螺仪弧度制转换
				
				#ifdef RM3510
				if(DBUS_Det(dbus))//rc开启判断
				{
				   move_control(dbus.rc.ch0, dbus.rc.ch1, dbus.rc.ch2, dbus.rc.s1, dbus.key.v,dbus.mouse.x);
				}
				#endif
				
			
				/*
				sprintf(buffer1,"%d \n",Get_Time_Micros()-a);
				a=Get_Time_Micros();
				printf(buffer1);
				*/
				
				
				EXTI_ClearFlag(EXTI_Line4);          
				EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3) == SET)
    {
        //75hz-13ms进中断一次（NOT USED），30hz-33ms进中断一次
        HMC5883L_ReadData();
				//AHRS计算
				AHRS_Calculate( 				 Gyro_Radian_Data.X,
																 Gyro_Radian_Data.Y,
																 Gyro_Radian_Data.Z,
																 MPU6050_Real_Data.Accel_X,
																 MPU6050_Real_Data.Accel_Y,
																 MPU6050_Real_Data.Accel_Z,
																 HMC5883L_Real_Data.Mag_X,
																 HMC5883L_Real_Data.Mag_Y,
																 HMC5883L_Real_Data.Mag_Z);
				
				//sprintf(buffer1,"%f \r %f \r %f \n",AHRS_Data.Pos_Nav_x,AHRS_Data.Pos_Nav_y,AHRS_Data.Pos_Nav_z);
				//sprintf(buffer1,"%f \r %f \r %f \n",AHRS_Data.Vel_Nav_x,AHRS_Data.Vel_Nav_y,AHRS_Data.Vel_Nav_z);
				//printf(buffer1);
				//sprintf(buffer1,"%d \r %d \r %d \n",HMC5883L_Raw_Data.Mag_X,HMC5883L_Raw_Data.Mag_Y,HMC5883L_Raw_Data.Mag_Z);
				//sprintf(buffer1,"%f \n",HMC5883L_Real_Data.Yaw_Angle);
				//printf(buffer1);
				//delay_ms(100);
				
				EXTI_ClearFlag(EXTI_Line3);          
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

