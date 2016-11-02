/************************************************************************************
  File Name     :  mpu6050_hmc5883l_i2c.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  用于模拟mpu6050和hmc5883l的i2c过程
										----I2C1----SCL----PB8---
										----I2C1----SDA----PB9---
-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			  i2c配置
2     1.2       6/29       gyf 					
3     1.3       6/29       yf 					  注释			   
************************************************************************************/
#include "main.h"

#define IIC_SCL_H()      GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define IIC_SCL_L()      GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define IIC_SDA_H()      GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define IIC_SDA_L()      GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define IIC_SDA_Read()   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)

void IIC_Delay(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a = 6;//6
		while(a--);
	}
}

//HEAT_Configuration-PB4
void HEAT_Configuration(void)
{
	GPIO_InitTypeDef gpio;   

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		
	gpio.GPIO_Pin = GPIO_Pin_4;	
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpio);
    GPIO_ResetBits(GPIOB,GPIO_Pin_4);//0
}

//IIC_GPIO_Init OUT-PB8 PB9
void IIC_GPIO_Init(void)
{
    GPIO_InitTypeDef   gpio;
    
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_OD;
		gpio.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_Init(GPIOB, &gpio);
}

//IIC_SDA_Out-PB9 OUT
void IIC_SDA_Out(void)
{
    GPIO_InitTypeDef   gpio;
    
		gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_Init(GPIOB, &gpio);
}

//IIC_SDA_In-PB9 IN
void IIC_SDA_In(void)
{
    GPIO_InitTypeDef   gpio;
    
	gpio.GPIO_Pin = GPIO_Pin_9;
    
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);
}

// IIC_Start
// SDA OUT -H
// SCL -H
// Delay
// SDA -L
// Delay
// SCL -L
void IIC_Start(void)								  
{
	IIC_SDA_Out();
	IIC_SDA_H();
	IIC_SCL_H();
	IIC_Delay(1);
	IIC_SDA_L();
	IIC_Delay(1);
	IIC_SCL_L();
}
// IIC_Stop
// SDA OUT 
// SCL -L
// SDA -L
// Delay
// SCL -H
// SDA -H
// Delay
void IIC_Stop(void)
{
	IIC_SDA_Out();
	IIC_SCL_L();
	IIC_SDA_L();
	IIC_Delay(1);
	IIC_SCL_H();
	IIC_SDA_H();
	IIC_Delay(1);
}
// IIC_Ack(u8 re)		
// SDA OUT
// SDA -(re)
// SCL -H
// Delay
// SCL -L
// Delay
void IIC_Ack(u8 re)					     
{
	IIC_SDA_Out();
	if(re)
	   IIC_SDA_H();
	else
	   IIC_SDA_L();
	IIC_SCL_H();
	IIC_Delay(1);
	IIC_SCL_L();
	IIC_Delay(1);
}
// IIC_WaitAck
// SDA -H
// SDA In
// Delay
// SCL -H
// Delay
// READ - 
//		OUT_TIME-- to IIC_Stop return 0xff 超时错误
// SCL -L return 0
int IIC_WaitAck(void)
{
	u16 Out_Time=1000;
    
    IIC_SDA_H();
	IIC_SDA_In();
	IIC_Delay(1);
	IIC_SCL_H();
	IIC_Delay(1);
	while(IIC_SDA_Read())
	{
		if(--Out_Time)
		{
			IIC_Stop();
            printf("error IIC_WaitAck\r\n");
            return 0xff;
		}
	}
	IIC_SCL_L();
    return 0;
}
// IIC_WriteBit(u8 Temp)
// SDA OUT
// SCL -L
// LOOP 8次 即temp从高位到低位
// 		SDA -(Temp第八位） 
// 		Temp左移一位
// 		Delay
// 		SCL -H
// 		Delay
// 		SCL -L
void IIC_WriteBit(u8 Temp)
{
	u8 i;
	IIC_SDA_Out();
	IIC_SCL_L();
	for(i=0;i<8;i++)
	{
		if(Temp&0x80)
		{
			IIC_SDA_H();
		}
		else
		{
			IIC_SDA_L();
		}
		Temp<<=1;
		IIC_Delay(1);
		IIC_SCL_H();
		IIC_Delay(1);
		IIC_SCL_L();
	}
}
// IIC_ReadBit
// SDA IN
// LOOP 8次
// 		SCL -L
// 		Delay 
// 		SCL -H 
// 		Temp左移一位
// 		READ -Temp++(阅读位)
//		Delay
// SCL -L
// RETURN Temp
u8 IIC_ReadBit(void)
{
	u8 i,Temp=0;
	IIC_SDA_In();
	for(i=0;i<8;i++)
	{
		IIC_SCL_L();
		IIC_Delay(1);
		IIC_SCL_H();
		Temp<<=1;
		if(IIC_SDA_Read())
		   Temp++;
		IIC_Delay(1);
	}
	IIC_SCL_L();
	return Temp;
}

//写数据，成功返回0，失败返回0xff
int IIC_WriteData(u8 dev_addr,u8 reg_addr,u8 data)
{
	IIC_Start();
    
	IIC_WriteBit(dev_addr);
	if(IIC_WaitAck() == 0xff)
    {
        printf("error IIC_WriteData_dev_WaitAck\r\n");
        return 0xff;
    }
    
	IIC_WriteBit(reg_addr);
	if(IIC_WaitAck() == 0xff)
    {
        printf("error IIC_WriteData_reg_WaitAck\r\n");
        return 0xff;
    }

    IIC_WriteBit(data);
    if(IIC_WaitAck() == 0xff)
    {
        printf("error IIC_WriteData_data_WaitAck\r\n");
        return 0xff;
    }

	IIC_Stop();
    return 0;
}

//读数据，成功返回0，失败返回0xff
int IIC_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count)
{
	u8 i;

    IIC_Start();
	
    IIC_WriteBit(dev_addr);
	if(IIC_WaitAck() == 0xff)
    {
        printf("error IIC_ReadData_dev_WaitAck\r\n");
        return 0xff;
    }
    
    IIC_WriteBit(reg_addr);
	if(IIC_WaitAck() == 0xff)
    {
        printf("error IIC_ReadData_reg_WaitAck\r\n");
        return 0xff;
    }
	
    IIC_Start();
    
    IIC_WriteBit(dev_addr+1);
	if(IIC_WaitAck() == 0xff)
    {
        printf("error IIC_ReadData_dev+1_WaitAck\r\n");
        return 0xff;
    }
    
    for(i=0;i<(count-1);i++)
    {
        *pdata=IIC_ReadBit();
        IIC_Ack(0);
        pdata++;
    }

    *pdata=IIC_ReadBit();
    IIC_Ack(1); 
    
    IIC_Stop(); 
    
    return 0;    
}
