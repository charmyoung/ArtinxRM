#ifndef __UNDERPAN_TASK_H
#define __UNDERPAN_TASK_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


/** 
  * @brief  底盘运动模式
  */
typedef enum
{
  StopMoveLock    = 0x00,
  StopMoveNotLock,
  MoveUnderRC,              //遥控模式
  MoveAuto,                 //Bezier曲线模式
  PID_Debug,                //调试PID
  MovePlay,                 //弹球模式
  MoveCircle,               //圆圈
  MoveFullAuto,             //终极全自动模式
  UpperComputerDebug,       //上位机调试
  
}MoveModeTypeDef;


/** 
  * @brief  X方向 和 Y方向
  */
enum
{
  X_Drct = 0,
  Y_Drct = 1,
};


/** 
  * @brief  在底盘运动任务中的数据
  */
typedef struct 
{
	float   fGlobalAngle;           //世界坐标系中底盘的角度
	int16_t s16_GlobalX;            //世界坐标系中底盘所处X坐标
  int16_t s16_GlobalY;            //世界坐标系中底盘所处Y坐标
  float   fSpeedX;                //x方向上的速度
  float   fSpeedY;                //y方向上的速度
  float   fOmega;                 //自转的角速度
  float   fTarAngle;              //目标角度，顺时针为正
  float   fTarDrct;               //车体目标运动方向（容易和fTarAngle搞混）
  int16_t s16_WheelSpd[4];        //发送给电机的速度
  int16_t s16_WheelSpdLmt;        //轮子限制速度
  MoveModeTypeDef emBaseMoveMode; //基地的模式
	/*new add*/
}UnderpanDataTypeDef;  //底盘数据结构


/** 
  * @brief  血量变化实时记录
  *       (变化方式，变化值
  *       变化装甲板ID（如果不是装甲板扣血则为-1）)
  *       变化时间（important）
  *       被打中时车头朝向
  */
typedef struct
{
  uint8_t   u8_Way;
  uint16_t  u16_Val;
  int8_t    s8_WeakId;
  uint32_t  u32_TimRcrd;
  float     fAngRcrd;
}BldChangeRcrdTypeDef;


/** 
  * @brief  三个数据记录
  */
enum
{
  HERO_RCRD = 0,
  UAV_RCRD,
  INF_RCRD,
  MANY_RCRD,
  RCRD_LEN
};


/** 
  * @brief  受到攻击管理
  */
typedef struct
{
  uint8_t   u8_EnemyHeroAppr;      //英雄出现
  uint8_t   u8_EnemyUAVAppr;       //无人机来了
  uint8_t   u8_EnemyInfAppr;       //步兵出现
  uint8_t   u8_TooManyEnemy;        //好多敌人
  BldChangeRcrdTypeDef  tAtckRcrd[RCRD_LEN];  //受到攻击血量变化记录
}AtckMngTypeDef;


/** 
  * @brief  功率管理
  */
typedef struct {
	float fAllPower;              //总功率 暂时不用
	int16_t s16_CurLimit;         //总电流限制 mA
	int16_t s16_AllWheelCur;      //四个轮子的总电流 mA
	int16_t s16_ResidueCur;       //剩余电流
  int16_t s16_GivenMaxCur[4];   //给定最大电流值
}PowerManageTypeDef;



void underpanTaskThreadCreate(osPriority taskPriority);

#endif


///**
//  * @brief  限制车移动的两个方向的加速度
//            我理一理：
//            1.X速度方向与其加速度方向一致(为正or负)，此时Y速度方向与其加速度方向一致或不一致
//            2.X速度方向与其加速度方向不一致(为正or负)，此时Y速度方向与其加速度方向一致或不一致
//            人生的经验告诉我：方向一致要算入总加速度，方向不一致基本可以不算入考虑
//  * @param  None
//  * @retval None
//  */
//static void LimitAcc(void)
//{
//  
////  static float lastSpdX = 0,lastSpdY = 0;
////  float xSpdDiff,ySpdDiff;
////  
//  xSpdDiff = g_tBaseUnderpan.fSpeedX-lastSpdX;
//  ySpdDiff = g_tBaseUnderpan.fSpeedY-lastSpdY;
//  
//  //X速度方向与加速度方向一致
//  if(xSpdDiff*g_tBaseUnderpan.fSpeedX>0)
//  {
//     //Y速度方向与加速度方向一致
//    if(ySpdDiff*g_tBaseUnderpan.fSpeedY>0)
//    {
//      testAcc=sqrt(SQUARE(xSpdDiff)+SQUARE(ySpdDiff));

//      if(testAcc>MAX_ACC)
//      {
//        xSpdDiff = 1.5f/testAcc*xSpdDiff;
//        ySpdDiff = 1.5f/testAcc*ySpdDiff;
//      }
//    }
//    //Y速度方向与加速度方向不一致
//    else
//    {
//      if(xSpdDiff>MAX_ACC)
//      {
//        xSpdDiff = MAX_ACC;
//      }
//      else if(xSpdDiff<-MAX_ACC)
//      {
//         xSpdDiff = -MAX_ACC;
//      }
//    }
//  }
//  //X速度方向与加速度方向不一致
//  else
//  {
//    //Y速度方向与加速度方向一致
//    if(ySpdDiff*g_tBaseUnderpan.fSpeedY>0)
//    {
//      //Y 都大于0
//      if(ySpdDiff>MAX_ACC)
//      {
//        ySpdDiff = MAX_ACC;
//      }
//      else if(ySpdDiff<-MAX_ACC)
//      {
//        ySpdDiff = -MAX_ACC;
//      }
//    }
//    //Y速度方向与加速度方向不一致
//    else
//    {
////      if(testAcc>5)
//      
////      {
////        xSpdDiff = MAX_ACC/testAcc*xSpdDiff;
////        ySpdDiff = MAX_ACC/testAcc*ySpdDiff;
////      }
//    }
//  }
//  g_tBaseUnderpan.fSpeedX=xSpdDiff+lastSpdX;
//  g_tBaseUnderpan.fSpeedY=ySpdDiff+lastSpdY;
//  
//  lastSpdX = g_tBaseUnderpan.fSpeedX;
//  lastSpdY = g_tBaseUnderpan.fSpeedY;
//}


///**
//  * @brief  自动移动_1号(弹球模式),不完善
//  * @param  None
//  * @retval None
//  */
//#define GetRandomAngle(sA, eA)  (HAL_RNG_GetRandomNumber(&hrng)/4294967296.0f*(eA - sA)+sA)
//float fAdvanceV = 3.6f;
//static void AutoMove_No_1(void)
//{
//  static float fAngle = 0,fLastAngle = 0;  
////  tUAV_Data.s16_UAV_AbsoluX = testTemp1;
////  tUAV_Data.s16_UAV_AbsoluY = testTemp2;
////  tUAV_Data.s16_UAV_AbsoluX = tUAV_Data.s16_UAV_RelaX;
////  tUAV_Data.s16_UAV_AbsoluY = tUAV_Data.s16_UAV_RelaY;

//  if(fAngle == 0)
//  {
//    fAngle = GetRandomAngle(0.0f, 2.0f * PI);
//    //获取当前位置
//    fMoveP[2][X_Drct] = tBaseMixLocData.s16_xMix;
//    fMoveP[2][Y_Drct] = tBaseMixLocData.s16_yMix;
//  }
//  fMoveP[1][X_Drct] = fMoveP[2][X_Drct];
//  fMoveP[1][Y_Drct] = fMoveP[2][Y_Drct];
//  //计算下一位置
//  fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//  fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//  
//  
//  //优先判断是否撞边
//  switch(((uint8_t)(fMoveP[2][X_Drct]>820)<<3)|
//          ((uint8_t)(fMoveP[2][X_Drct]<-820)<<2)|
//          ((uint8_t)(fMoveP[2][Y_Drct]>820)<<1)|
//          ((uint8_t)(fMoveP[2][Y_Drct]<-820)<<0))
//  {
//    case 0x08://1000 右
//    case 0x0A://1010 右上
//      if(fMoveP[2][Y_Drct] > 600)
//      {
//        fAngle = GetRandomAngle(PI,1.4f*PI);
//      }
//      else if(fMoveP[2][Y_Drct] < -600)
//      {
//        fAngle = GetRandomAngle(0.6f*PI,PI);
//      }
//      else
//      {
//        fAngle = GetRandomAngle(0.6f*PI, 1.4f*PI);
//      }
//      fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//      fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//      break;
//    
//    case 0x02://0010 上
//    case 0x06://0110 左上
//      if(fMoveP[2][X_Drct] > 600)
//      {
//        fAngle = GetRandomAngle(1.1f*PI,1.5f*PI);
//      }
//      else if(fMoveP[2][X_Drct] < -600)
//      {
//        fAngle = GetRandomAngle(1.5f*PI,1.9f*PI);
//      }
//      else
//      {
//        fAngle = GetRandomAngle(1.1f*PI,1.9f*PI);
//      }
//      fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//      fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//      break;
//    
//    case 0x04://0100 左
//    case 0x05://0101 左下
//      if(fMoveP[2][Y_Drct] > 600)
//      {
//        fAngle = GetRandomAngle(1.6f*PI,2.0f*PI);
//      }
//      else if(fMoveP[2][Y_Drct] < -600)
//      {
//        fAngle = GetRandomAngle(2.0f*PI,2.4f*PI);
//      }
//      else
//      {
//        fAngle = GetRandomAngle(1.6f*PI,2.4f*PI);
//      }
//      fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//      fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//      break;
//    
//    case 0x01://0001 下
//    case 0x09://1001 右下
//      if(fMoveP[2][X_Drct] > 600)
//      {
//        fAngle = GetRandomAngle(0.5f*PI,0.9f*PI);
//      }
//      else if(fMoveP[2][X_Drct] < -600)
//      {
//        fAngle = GetRandomAngle(0.1f*PI,0.5f*PI);
//      }
//      else
//      {
//        fAngle = GetRandomAngle(0.1f*PI,0.9f*PI);
//      }
//      fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//      fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//      break;
//    
//    default:
//      //判断位置
//      switch(((uint8_t)(fMoveP[2][X_Drct]<tUAV_Data.s16_UAV_AbsoluX)<<1)|
//              ((uint8_t)(fMoveP[2][Y_Drct]<tUAV_Data.s16_UAV_AbsoluY)))
//      {
//        case 0x00:/*右上*/
//          switch(((uint8_t)(fMoveP[2][X_Drct]<tUAV_Data.s16_UAV_AbsoluX+UAV_RANGE)<<1)|
//                  ((uint8_t)(fMoveP[2][Y_Drct]<tUAV_Data.s16_UAV_AbsoluY+UAV_RANGE)))
//          {
//            case 0x00:/*右上部分，完全不相交*/
//            case 0x01:
//            case 0x02:
//              break;
//            case 0x03:
//              if(abs(tUAV_Data.s16_UAV_AbsoluY-fMoveP[2][Y_Drct])>abs(tUAV_Data.s16_UAV_AbsoluX-fMoveP[2][X_Drct]))
//              {
//                fAngle = GetRandomAngle(PI*0.1f,PI*0.9f);
//              }
//              else
//              {
//                fAngle = GetRandomAngle(-PI*0.4f,PI*0.4f);
//              }
//              fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//              fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//              break;
//          }
//          break;
//        case 0x02:/*左上*/
//          switch(((uint8_t)(fMoveP[2][X_Drct]>tUAV_Data.s16_UAV_AbsoluX-UAV_RANGE)<<1)|
//                  ((uint8_t)(fMoveP[2][Y_Drct]<tUAV_Data.s16_UAV_AbsoluY+UAV_RANGE)))
//          {
//            case 0x00:/*不相交*/
//            case 0x01:
//            case 0x02:
//              break;
//            case 0x03:
//              if(abs(tUAV_Data.s16_UAV_AbsoluY-fMoveP[2][Y_Drct])>abs(tUAV_Data.s16_UAV_AbsoluX-fMoveP[2][X_Drct]))
//              {
//                fAngle = GetRandomAngle(PI*0.1f,PI*0.9f);
//              }
//              else
//              {
//                fAngle = GetRandomAngle(PI*0.6f,PI*1.4f);
//              }
//              fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//              fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//              break;
//          }
//          break;
//        case 0x01:/*右下*/
//          switch(((uint8_t)(fMoveP[2][X_Drct]<tUAV_Data.s16_UAV_AbsoluX+UAV_RANGE)<<1)|
//                  ((uint8_t)(fMoveP[2][Y_Drct]>tUAV_Data.s16_UAV_AbsoluY-UAV_RANGE)))
//          {
//            case 0x00:/*不相交*/
//            case 0x01:
//            case 0x02:
//              break;
//            case 0x03:
//              if(abs(tUAV_Data.s16_UAV_AbsoluY-fMoveP[2][Y_Drct])>abs(tUAV_Data.s16_UAV_AbsoluX-fMoveP[2][X_Drct]))
//              {
//                fAngle = GetRandomAngle(PI*1.1f,PI*1.9f);
//              }
//              else
//              {
//                fAngle = GetRandomAngle(-PI*0.4f,PI*0.4f);
//              }
//              fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//              fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//              break;
//          }
//          break;
//        case 0x03:/*左下*/
//          switch(((uint8_t)(fMoveP[2][X_Drct]>tUAV_Data.s16_UAV_AbsoluX-UAV_RANGE)<<1)|
//                  ((uint8_t)(fMoveP[2][Y_Drct]>tUAV_Data.s16_UAV_AbsoluY-UAV_RANGE)))
//          {
//            case 0x00:/*不相交*/
//            case 0x01:
//            case 0x02:
//              break;
//            case 0x03:
//              if(abs(tUAV_Data.s16_UAV_AbsoluY-fMoveP[2][Y_Drct])>abs(tUAV_Data.s16_UAV_AbsoluX-fMoveP[2][X_Drct]))
//              {
//                fAngle = GetRandomAngle(PI*1.1f,PI*1.9f);
//              }
//              else
//              {
//                fAngle = GetRandomAngle(PI*0.6f,PI*1.4f);
//              }
//              fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+cos(fAngle)*fAdvanceV;
//              fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+sin(fAngle)*fAdvanceV;
//              break;
//          }
//          break;
//      }
//      break;
//  }
//  CalVxVy(fMoveP[2]);
////  LimitAcc();
//  //更新！
//  if(fAngle != fLastAngle)
//  {
//    fLastAngle = fAngle;
//    fAdvanceV = 2.4f;
//  }
//  else
//  {
//    if(fAdvanceV <= 3.6f)
//      fAdvanceV += 0.01f;
//  }
//}

