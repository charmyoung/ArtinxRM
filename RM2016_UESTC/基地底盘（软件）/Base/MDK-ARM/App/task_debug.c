/**
  ******************************************************************************
  * @file    task_debug.c
  * @author  
  * @version V1.1
  * @date    2016/05/31
  * @brief   调试任务，加上串口发送上位机
  * 
  ******************************************************************************
  * @attention 
  *			
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "task_debug.h"
#include "usart.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_pid.h"
#include "judgement_libs.h"
/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
osThreadId  debugTaskHandle;

uint8_t debugFrame[FrameLength];
float debugPos[2];    //接收发过来的坐标


//收到的正确的帧计数
uint16_t frame_cnt;
FloatConvertType gTmp;

extern uint8_t arrBT_RxBuf[BT_RXBUF_LEN];
extern MixLocDataTypeDef    tBaseMixLocData;
extern RadarDataTypeDef     tBaseRadarData;
extern GyroDataTypeDef      tBaseGyroData;
extern float   fMoveP[3][2];   //移动要经过的路径点（Move point）
extern int16_t s16_CalP[4][2];
extern PID_TypeDef tPID_Angle;       //底盘角度pid
extern PID_TypeDef tPID_VX;          //X方向上的速度
extern PID_TypeDef tPID_VY;          //Y方向上的速度
extern tGameInfo testGameInfo;
extern tRealShootData testRealShootData;
extern UAV_DataTypeDef tUAV_Data;


void SendParament(int16_t id,int value);

/**
  * @brief  调试任务
  * @param  void const * argument
  * @retval None
  */
void DebugTask(void const * argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    
    static uint8_t i=0;
    osDelayUntil(&xLastWakeTime, 20);
//    debugSendFrame(&BLUET_HUART, Monitor4, (float)tBaseMixLocData.s16_xMix);
//    debugSendFrame(&BLUET_HUART, Monitor5, (float)tBaseMixLocData.s16_yMix);
//    debugSendFrame(&BLUET_HUART, Monitor6, (float)tBaseMixLocData.fAngleMix);
    
    //计算出来的运动轨迹点
    debugSendFrame(&BLUET_HUART, Monitor1, fMoveP[2][0]);
    debugSendFrame(&BLUET_HUART, Monitor2, fMoveP[2][1]);
    
//    debugSendFrame(&BLUET_HUART, Monitor1, tBaseRadarData.s16_RadarX);
//    debugSendFrame(&BLUET_HUART, Monitor2, tBaseRadarData.s16_RadarY);
//    if(i == 0){
//      gTmp.s16Type[0] = tUAV_Data.s16_UAV_RelaX+300;
//      gTmp.s16Type[1] = tUAV_Data.s16_UAV_RelaY+300;
//    }else if(i == 1){
//      gTmp.s16Type[0] = tUAV_Data.s16_UAV_RelaX+300;
//      gTmp.s16Type[1] = tUAV_Data.s16_UAV_RelaY-300;
//    }else if(i == 2){
//      gTmp.s16Type[0] = tUAV_Data.s16_UAV_RelaX-300;
//      gTmp.s16Type[1] = tUAV_Data.s16_UAV_RelaY-300;
//    }else{
//      gTmp.s16Type[0] = tUAV_Data.s16_UAV_RelaX-300;
//      gTmp.s16Type[1] = tUAV_Data.s16_UAV_RelaY+300;
//    }
  
    if(i == 0){
      gTmp.s16Type[0] = tUAV_Data.s16_UAV_AbsoluX+300;
      gTmp.s16Type[1] = tUAV_Data.s16_UAV_AbsoluY+300;
    }else if(i == 1){
      gTmp.s16Type[0] = tUAV_Data.s16_UAV_AbsoluX+300;
      gTmp.s16Type[1] = tUAV_Data.s16_UAV_AbsoluY-300;
    }else if(i == 2){
      gTmp.s16Type[0] = tUAV_Data.s16_UAV_AbsoluX-300;
      gTmp.s16Type[1] = tUAV_Data.s16_UAV_AbsoluY-300;
    }else{
      gTmp.s16Type[0] = tUAV_Data.s16_UAV_AbsoluX-300;
      gTmp.s16Type[1] = tUAV_Data.s16_UAV_AbsoluY+300;
    }
    
    debugSendFrame(&BLUET_HUART, Monitor5, gTmp.fType);

    i++;
    i = i%4;

    //实际的运动轨迹
    debugSendFrame(&BLUET_HUART, Monitor3, (float)tBaseMixLocData.s16_xMix);
    debugSendFrame(&BLUET_HUART, Monitor4, (float)tBaseMixLocData.s16_yMix);
    //功率值
//    debugSendFrame(&BLUET_HUART, Monitor5, testGameInfo.realChassisOutA*testGameInfo.realChassisOutV);
    //射速
//    debugSendFrame(&BLUET_HUART, Monitor5, testRealShootData.realGolfShootFreq);
//    debugSendFrame(&BLUET_HUART, Monitor6, testRealShootData.realGolfShootSpeed);

    //雷达坐标和角度
    debugSendFrame(&BLUET_HUART, Monitor7, tBaseRadarData.s16_RadarX);
    debugSendFrame(&BLUET_HUART, Monitor8, tBaseRadarData.s16_RadarY);
    debugSendFrame(&BLUET_HUART, Monitor6, tBaseRadarData.s8_RadarAngle);
    
    //angle pid
//    debugSendFrame(&BLUET_HUART, GyroKp, tPID_Angle.kp);
//    debugSendFrame(&BLUET_HUART, GyroKi, tPID_Angle.ki);
//    debugSendFrame(&BLUET_HUART, GyroKd, tPID_Angle.kd);
    
//    UART_Printf(&BLUET_HUART,"X:%d,Y:%d,Angle:%f\r\n" ,tBaseMixLocData.s16_xMix,tBaseMixLocData.s16_yMix,tBaseMixLocData.fAngleMix);
//    SendParament(1, tBaseMixLocData.s16_xMix);
//    SendParament(2, tBaseMixLocData.s16_yMix);
//    SendParament(3, tBaseMixLocData.fAngleErr);
//    
//    SendParament(4, (int16_t)fMoveP[2][0]);
//    SendParament(5, (int16_t)fMoveP[2][1]);
  }
}


/**
  * @brief  发送给小明的上位机
  * @param  
  * @retval None
  */
void debugSendFrame(UART_HandleTypeDef *huart, RcTableType id, float data)
{
  FloatConvertType	tmp;
  debugFrame[Head1]   = 0x55;
  debugFrame[Head2]   = 0xAA;
  debugFrame[DataID]  = id;
 
  tmp.fType = data;

  debugFrame[Byte0]   = tmp.u8Type[0];
  debugFrame[Byte1]   = tmp.u8Type[1];
  debugFrame[Byte2]   = tmp.u8Type[2];
  debugFrame[Byte3]   = tmp.u8Type[3];
  
  debugFrame[SumCheck]=   (uint8_t)(debugFrame[DataID] 
                                      + debugFrame[Byte0] 
                                      + debugFrame[Byte1]  
                                      + debugFrame[Byte2] 
                                      + debugFrame[Byte3]); 
  debugFrame[Tail]    = 0xFF;
  
  HAL_UART_Transmit(huart, debugFrame, FrameLength,100);  
  //use blocking mode transmit
}


/**
  * @brief  蓝牙接收回调
  * @param  None
  * @retval None
  */
void BT_RxDataHandler(void)
{
  FloatConvertType	tmp;
  
  if( arrBT_RxBuf[Head1] == 0x55 && 
			arrBT_RxBuf[Head2] == 0xAA && 
      arrBT_RxBuf[Tail]  == 0xFF)
  {
    frame_cnt ++;
    
    uint8_t id = arrBT_RxBuf[DataID] ;
    uint8_t sum =  (uint8_t)( arrBT_RxBuf[DataID] 
                + arrBT_RxBuf[Byte0] 
                + arrBT_RxBuf[Byte1] 
                + arrBT_RxBuf[Byte2] 
                + arrBT_RxBuf[Byte3]);
    if(sum == arrBT_RxBuf[SumCheck])
    {	
      tmp.u8Type[0] = arrBT_RxBuf[Byte0];
      tmp.u8Type[1] = arrBT_RxBuf[Byte1];
      tmp.u8Type[2] = arrBT_RxBuf[Byte2];
      tmp.u8Type[3] = arrBT_RxBuf[Byte3];
      
      switch(id)
      {
//        case GyroKp:
//          tPID_Angle.kp = tmp.fType;
//          break;
//        case GyroKi:
//          tPID_Angle.ki = tmp.fType;
//          break;
//        case GyroKd:
//          tPID_Angle.kd = tmp.fType;
//          break;
        //目标位置
        case LandingStatus:
          debugPos[0] = tmp.s16Type[0];
          debugPos[1] = tmp.s16Type[1];
          
      }
      debugSendFrame(&BLUET_HUART,CmdMcuGotParamOK, 12);
      UART_Printf(&BLUET_HUART, "OK!!!!!!!!!!!!!!!!!");
    }
  }
}


/**
  * @brief  Create the DebugTask threads
  * @param  None
  * @retval None
  */
void debugTaskThreadCreate(osPriority taskPriority)
{
	osThreadDef(debugTask, DebugTask, taskPriority, 0, 256);
  debugTaskHandle = osThreadCreate(osThread(debugTask), NULL);
}
