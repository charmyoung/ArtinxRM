/************************************************************************************
  File Name     :  sdbus.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  sdbus编码和解码，仅用到了解码
-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.6       7/8        yf   			    sdbus       编码解码	   
************************************************************************************/
#include "main.h"
int SDFlag=0;//默认为手动
int LastQKey=0;//上一个Q
int LastEKey=0;
int BlueFlag=0;//默认为红色

void SDState_Set(uint16_t v)
{
	int key_Q = KEY_PRESSED_OFFSET_Q & v; 
  int key_E = KEY_PRESSED_OFFSET_E & v; 
	if (key_Q!=0) key_Q=1;
	if (key_E!=0) key_E=1;
	if (LastQKey==0&&key_Q==1)
		{		
			SDFlag=1-SDFlag;
			if(SDFlag==0)
			{
				Yaw.targetAngle=YAW_MID;
				Pitch.targetAngle=PITCH_MID;	
			}
		}
	if (LastQKey==0&&key_Q==1)
	{
		Yaw.targetAngle=YAW_MID;
		Pitch.targetAngle=PITCH_MID;	
	}
	LastQKey=key_Q;
	LastEKey=key_E;
}


void SD_TriggerControl(void)
{
			//Pitch轴
			Pitch.targetAngle += ((float)sdbus.PitchAngle/360.)*8192; 
			if (Pitch.targetAngle < Pitch.minAngle){Pitch.targetAngle=Pitch.minAngle;}
			if (Pitch.targetAngle > Pitch.maxAngle){Pitch.targetAngle=Pitch.maxAngle;}		
			//Yaw轴
			Yaw.targetAngle += ((float)sdbus.YawAngle/360.)*8192;
			if (Yaw.targetAngle < Yaw.minAngle){Yaw.targetAngle=Yaw.minAngle;}
			if (Yaw.targetAngle > Yaw.maxAngle){Yaw.targetAngle=Yaw.maxAngle;}
				
}

SDBUS sdbus;
void SDBUS_Enc(const SDBUS* sdbus,unsigned char* sdbuf)//sdbus编码
{
    sdbuf[0] = (sdbus->PitchAngle > 0)+'0';
    sdbuf[1] =  abs(sdbus->PitchAngle)/10+'0';
		sdbuf[2] = abs(sdbus->PitchAngle)%10+'0';
		sdbuf[3] = (sdbus->YawAngle > 0)+'0';
		sdbuf[4] = abs(sdbus->YawAngle)/10+'0';
		sdbuf[5] = abs(sdbus->YawAngle)%10+'0';
}

void SDBUS_Dec(SDBUS* sdbus,const unsigned char* sdbuf)//sdbus解码
{
    sdbus->PitchAngle = (2*(sdbuf[0]-'0')-1)*((sdbuf[1]-'0')*10+(sdbuf[2]-'0'));
    sdbus->YawAngle= (2*(sdbuf[3]-'0')-1)*((sdbuf[4]-'0')*10+(sdbuf[5]-'0'));
}
