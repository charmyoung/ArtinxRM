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


SDBUS sdbus={0,0,0};
void SDBUS_Enc(const SDBUS* sdbus,unsigned char* sdbuf)//sdbus编码
{
    /*sdbuf[0] = (sdbus->PitchAngle > 0)+'0';
    sdbuf[1] =  abs(sdbus->PitchAngle)/10+'0';
		sdbuf[2] = abs(sdbus->PitchAngle)%10+'0';
		sdbuf[3] = (sdbus->YawAngle > 0)+'0';
		sdbuf[4] = abs(sdbus->YawAngle)/10+'0';
		sdbuf[5] = abs(sdbus->YawAngle)%10+'0';
		*/
}

void SDBUS_Dec(SDBUS* sdbus,const unsigned char* sdbuf)//sdbus解码
{
    //sdbus->PitchAngle = (2*(sdbuf[0]-'0')-1)*((sdbuf[1]-'0')*10+(sdbuf[2]-'0'));
    //sdbus->YawAngle= (2*(sdbuf[3]-'0')-1)*((sdbuf[4]-'0')*10+(sdbuf[5]-'0'));
		sdbus->xf = (double)( (','-sdbuf[0])*((sdbuf[1]-'0')*100+(sdbuf[2]-'0')*10+(sdbuf[3]-'0')) )*0.4;
		sdbus->xtr = (double)( (','-sdbuf[4])*((sdbuf[5]-'0')*100+(sdbuf[6]-'0')*10+(sdbuf[7]-'0')) )*0.4;
		sdbus->xrr = (double)( (','-sdbuf[8])*((sdbuf[9]-'0')*100+(sdbuf[10]-'0')*10+(sdbuf[11]-'0')) )*0.4;
		sdbus->w1 = RM3510_1.thisPosition+sdbus->xf+sdbus->xtr+sdbus->xrr;
		sdbus->w2 = -RM3510_2.thisPosition+sdbus->xf-sdbus->xtr-sdbus->xrr;
		sdbus->w3 = RM3510_3.thisPosition+sdbus->xf-sdbus->xtr+sdbus->xrr;
		sdbus->w4 = -RM3510_4.thisPosition+sdbus->xf+sdbus->xtr-sdbus->xrr;
}
