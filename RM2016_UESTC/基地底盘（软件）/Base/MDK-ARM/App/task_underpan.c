/**
  ******************************************************************************
  * @file    task_underpan.c
  * @author  
  * @version V1.1
  * @date    2016/01/22
  * @brief   底盘运行任务
  * 
  ******************************************************************************
  * @attention 
  *			
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "task_underpan.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_pid.h"
#include "task_bulletLoading.h"
#include "timer_canSender.h"
#include "task_detect.h"
#include "task_keyPress.h"
#include "judgement_libs.h"
#include "includes.h"
/* Defines -------------------------------------------------------------------*/
#define UNDERPAN_PERIOD     5         //周期为5ms
#define MAX_WHEEL_SPEED     950       //轮子最大速度
#define AUTO_WHEEL_SPD_MAX  500       //自动移动速度
#define ALPHA               45        //鬼角度
#define MATCH_RATIO         (1/5.8f)  //拟合系数
#define UAV_RANGE           300       //无人机覆盖范围，边长的1/2
#define CUR_LMT             2600      //初始最大限制电流（绝对不超功率，不管电池电量变化）

//装甲编号
#define ARMOUR_F      0
#define ARMOUR_L      1
#define ARMOUR_B      2
#define ARMOUR_R      3

#define MAX_ACC   3.00f //最大加速度
//创建一个由最小值和长度确定的随机数
#define CreateRandNum(min, len) (min+len*(HAL_RNG_GetRandomNumber(&hrng))/4294967296.f)
#define RNG_GETNUM()  ((HAL_RNG_GetRandomNumber(&hrng))/4294967296.f)   //[0,1]
#define SQUARE(x)   (x)*(x)   //平方
#define GET_HPTNS(x1,y1,x2,y2)  sqrtf(SQUARE(x1-x2)+SQUARE(y1-y2)) //hypotenuse(求斜边长度)
/* Variables -----------------------------------------------------------------*/
osThreadId underpanTaskHandle;

portTickType xLastWakeTime;
UnderpanDataTypeDef g_tBaseUnderpan;
PowerManageTypeDef g_tPowerManage;
PID_TypeDef tPID_Angle;       //底盘角度pid
PID_TypeDef tPID_VX;          //X方向上的速度
PID_TypeDef tPID_VY;          //Y方向上的速度
AtckMngTypeDef tBaseAM;       //基地血量变化管理

int16_t s16_CalP[4][2] = {{0,0},{400,500},{600,300},{200,100}};  //用于计算路径的初始点
float   fXminPoint[2], fXmaxPoint[2];

float   fMoveP[3][2];   //移动要经过的路径点（Move point）
float   t = 0;          //RatioRunned   跑过的比例

////test......................................
float fRC_Sensitivity = 1.4f;             //遥控器灵敏度
float testXminP[2]={800,-800},testXmaxP[2]={800,800};
//float testTemp1;
//float testTemp2;

extern RNG_HandleTypeDef hrng;
extern CAN_HandleTypeDef hcan1;
extern RC_TypeDef tRC_Data;
extern MixLocDataTypeDef tBaseMixLocData;
extern UAV_DataTypeDef tUAV_Data;
extern MotorDataTypeDef tMotorData[4];
extern int16_t debugPos[2];
extern GyroDataTypeDef      tBaseGyroData;
extern LoadingStatusTypeDef  tBaseLdSt;
extern __IO float fFixedMovePoint[][2];
extern JudgementDataTypedef tJData;
extern int8_t s8_LoadingStuckFlg;
extern KEYENCE_StateTypeDef  KS_List[KEYENCE_LIST_LEN];
extern uint8_t u8_KS_Symbol;
extern UpperDataTyperDef     tUpperData;
extern GameStartTypeDef      tGameStart; //比赛开始


//Debug
//static void MoveDebugPID(void);

static void MoveModeSelect(void);
static void UnderpanMoveControl(void);
static void CalculateWheelSpeed(float vx, float vy, float omega, float radian, int16_t maxspeed);
//static void MoveAlongTracePoint(uint16_t range);
static void CalVxVy(float* point);
//static void ResetGyro(void);
//static void ResetRadar(void);
static void Underpan_PID_Configuration(void);
static void CircleRun(int16_t r);
static void CalWheelSpdUnderPowerManage(float vx, float vy, float omega, float tarAngle);
//static void RepellentMode(void);
static void AllAutoMove(void);
static void RandomSpin(void);
//static void MoveInLineSegment(float* pfXmin, float* pfXmax);
//static void ArmourInjuredRun(AtckMngTypeDef* p_tBaseAM, uint16_t u16_Range);
static void RunLikeDancing(int16_t tranSpd, int16_t rttSpd, uint8_t Mode);


/**
  * @brief  底盘运动任务
  * @param  void const * argument
  * @retval None
  */
void UnderpanTask(void const * argument)
{
  osDelay(100);
  Underpan_PID_Configuration();
  g_tPowerManage.s16_CurLimit = CUR_LMT;
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    osDelayUntil(&xLastWakeTime, UNDERPAN_PERIOD);
    MoveModeSelect();
    UnderpanMoveControl();
  }
}


/**
  * @brief  底盘运动模式选择
  * @param  None
  * @retval None
  */
static void MoveModeSelect(void)
{
  int8_t nowLeftSW, nowRightSW;
  static int8_t lastLeftSW = -1, lastRightSW = -1;
  
  nowLeftSW = tRC_Data.switch_left;
  nowRightSW = tRC_Data.switch_right;
  
  if(lastLeftSW!=nowLeftSW \
    || lastRightSW!=nowRightSW)
  {
    switch(nowLeftSW)
    {
      case Switch_Up:
        switch(nowRightSW)
        {
          case Switch_Up:
            g_tBaseUnderpan.emBaseMoveMode = MoveUnderRC;
            break;
          
          case Switch_Middle:
            g_tBaseUnderpan.emBaseMoveMode = StopMoveLock;
            break;
          
          case Switch_Down:
//            ResetGyro();
//            osDelay(2500);
//            tBaseMixLocData.fAngleErrEf = 0;
//            xLastWakeTime = xTaskGetTickCount();
//            CHANGE_MIAXDATASRC2(M_AND_R);
//            g_tBaseUnderpan.emBaseMoveMode = MoveAuto;
            g_tBaseUnderpan.emBaseMoveMode = MoveFullAuto;
            break;
        }
        break;
        
      case Switch_Middle:
        switch(nowRightSW)
        {
          case Switch_Up:
            g_tBaseUnderpan.emBaseMoveMode = MoveCircle;
            break;
          
          case Switch_Middle:
            g_tBaseUnderpan.emBaseMoveMode = PID_Debug;
            break;
          
          case Switch_Down:
            g_tBaseUnderpan.emBaseMoveMode = MovePlay;
            break;
        }
        break;
        
      case Switch_Down:
        switch(nowRightSW)
        {
          case Switch_Up:
            g_tBaseUnderpan.emBaseMoveMode = UpperComputerDebug;
            break;
          
          case Switch_Middle:
            g_tBaseUnderpan.emBaseMoveMode = StopMoveNotLock;
            break;
          
          case Switch_Down:
            g_tBaseUnderpan.emBaseMoveMode = MoveFullAuto;
            break;
        }
        break;
        
      case No_Data:
        switch(nowRightSW)
        {
          case No_Data:
            if(tRC_Data.tSelfCheck.dataState == OFFLINE)
            {
              g_tBaseUnderpan.emBaseMoveMode = MoveFullAuto;
//              tBaseGyroData.fInitAngle = 90;
            }
            break;
          default:
            break;
        }
        break;
    }
    //开关值更新
    lastLeftSW  = nowLeftSW;
    lastRightSW = nowRightSW;
  }
}


/**
  * @brief  底盘运动控制
  * @param  None
  * @retval None
  */
static void UnderpanMoveControl(void)
{
  switch(g_tBaseUnderpan.emBaseMoveMode)
  {
    case StopMoveLock:
      g_tBaseUnderpan.fSpeedX = 0;
      g_tBaseUnderpan.fSpeedY = 0;
      g_tBaseUnderpan.fOmega  = 0;
      break;
    
    case MoveUnderRC:
      g_tBaseUnderpan.fTarDrct = tBaseMixLocData.fAngleMix;
      g_tBaseUnderpan.fSpeedX = tRC_Data.ch1*fRC_Sensitivity;
      g_tBaseUnderpan.fSpeedY = tRC_Data.ch2*fRC_Sensitivity;
      g_tBaseUnderpan.fOmega  = tRC_Data.ch3*fRC_Sensitivity;
      g_tBaseUnderpan.s16_WheelSpdLmt = MAX_WHEEL_SPEED;
      break;
    
    case MoveAuto:
//      g_tBaseUnderpan.fTarDrct = 45;
//      MoveAlongTracePoint(1550);
//      RepellentMode();
//      ArmourInjuredRun(tBaseAM, &tBaseAM,700);
//      MoveInLineSegment(testXminP, testXmaxP);
      RunLikeDancing(500,500,0);
//      g_tBaseUnderpan.s16_WheelSpdLmt = AUTO_WHEEL_SPD_MAX;
      break;
    
    case PID_Debug:
//      MoveDebugPID();
      break;
    
    case MovePlay:
      g_tBaseUnderpan.s16_WheelSpdLmt = MAX_WHEEL_SPEED;
      RandomSpin();
      break;
    
    case MoveCircle:
      CircleRun(650);
      CalVxVy(fMoveP[2]);
      break;
    
    case UpperComputerDebug:
      CalVxVy((float *)debugPos);
      break;
    
    case MoveFullAuto:
      g_tBaseUnderpan.s16_WheelSpdLmt = MAX_WHEEL_SPEED;
      AllAutoMove();
      break;
    
    default:
      break;
  }
  CalWheelSpdUnderPowerManage(g_tBaseUnderpan.fSpeedX, \
                              g_tBaseUnderpan.fSpeedY, \
                              g_tBaseUnderpan.fOmega, \
                              g_tBaseUnderpan.fTarDrct);
}


/**
  * @brief  根据3个速度值计算轮子的速度
  * @param  float vx X方向上的速度
  * @param  float vy Y方向上的速度
  * @param  float omega 旋转的速度
  * @param  float angle 此时基地所处的角度（顺时针为正）
  * @param  int16_t maxspped  最大速度
  * @retval None
  */
static void CalculateWheelSpeed(float vx, float vy, float omega, float radian, int16_t maxspeed)
{
  float   fMaxSpd = 0;
  float   fWheelSpd[4];
  int16_t __packed s16_WheelSpd[4];
  fWheelSpd[W_Right] = -vy*cos(radian)-vx*sin(radian)+omega;
  fWheelSpd[W_Front] = -vx*cos(radian)+vy*sin(radian)-omega;
  fWheelSpd[W_Left] = -(-vy*cos(radian)-vx*sin(radian)-omega);
  fWheelSpd[W_Behind] = -(-vx*cos(radian)+vy*sin(radian)+omega);
  
  fMaxSpd = abs(fWheelSpd[W_Right]);
  if(abs(fWheelSpd[W_Front]) > fMaxSpd)
		fMaxSpd = abs(fWheelSpd[W_Front]);
	if(abs(fWheelSpd[W_Left]) > fMaxSpd)
		fMaxSpd = fWheelSpd[W_Left];
	if(abs(fWheelSpd[W_Behind]) > fMaxSpd)
		fMaxSpd = fWheelSpd[W_Behind];
  
  //按比例限制速度
  if(fMaxSpd > maxspeed)
  {
		s16_WheelSpd[W_Right]   = (int16_t)(fWheelSpd[W_Right]*maxspeed/fMaxSpd);
		s16_WheelSpd[W_Front]   = (int16_t)(fWheelSpd[W_Front]*maxspeed/fMaxSpd);
		s16_WheelSpd[W_Left]    = (int16_t)(fWheelSpd[W_Left]*maxspeed/fMaxSpd);
		s16_WheelSpd[W_Behind]  = (int16_t)(fWheelSpd[W_Behind]*maxspeed/fMaxSpd);
	}
  else
  {
    s16_WheelSpd[W_Right]   = (int16_t)fWheelSpd[W_Right];
		s16_WheelSpd[W_Front]   = (int16_t)fWheelSpd[W_Front];
		s16_WheelSpd[W_Left]    = (int16_t)fWheelSpd[W_Left];
		s16_WheelSpd[W_Behind]  = (int16_t)fWheelSpd[W_Behind];
  }
  //强行增加主控负担
  memcpy((void*)g_tBaseUnderpan.s16_WheelSpd, (void*)s16_WheelSpd, 8);
}


/**
  * @brief  Create a new point
  * @param  uint16_t range(单位:mm)
  * @retval None
  */
//static void CreateNewPoint(uint16_t range)
//{ 
//  //memcpy 作用同上
//  memcpy(s16_CalP,s16_CalP+1,sizeof(uint16_t [3][3]));
//  
//  float fAngle  = atan2((s16_CalP[1][Y_Drct]-s16_CalP[2][Y_Drct]),(s16_CalP[1][X_Drct]-s16_CalP[2][X_Drct]));
//  float fK1     = tan(fAngle - ALPHA / 180.f * 3.14f);
//  float fK2     = tan(fAngle + ALPHA / 180.f * 3.14f);
//  float tmp1 = fK1*s16_CalP[1][X_Drct] - fK1*s16_CalP[2][X_Drct] + s16_CalP[2][Y_Drct] - s16_CalP[1][Y_Drct];
//  float tmp2 = fK2*s16_CalP[1][X_Drct] - fK2*s16_CalP[2][X_Drct] + s16_CalP[2][Y_Drct] - s16_CalP[1][Y_Drct];
//  
//  while(1)
//  {
//    s16_CalP[3][X_Drct] = (RNG_GETNUM()-0.5f) * range;
//    s16_CalP[3][Y_Drct] = (RNG_GETNUM()-0.5f) * range;
//    
//    float tmp3 = fK1*s16_CalP[3][0] - fK1*s16_CalP[2][0] + s16_CalP[2][1] - s16_CalP[3][1];
//    float tmp4 = fK2*s16_CalP[3][0] - fK2*s16_CalP[2][0] + s16_CalP[2][1] - s16_CalP[3][1];
//    if (//(tmp3*tmp1 < 0 || tmp4*tmp2 < 0) && 
//    (GET_HPTNS(s16_CalP[3][X_Drct],s16_CalP[3][Y_Drct],s16_CalP[2][X_Drct],s16_CalP[2][Y_Drct])>700))
//    {
//      t = 0;
//      break;
//    }
//  }
//}


/**
  * @brief  Update Cureve Trace //更新曲线轨迹得到要移动的目标点
  * @param  uint16_t range(单位:mm)
  * @retval None
  */
//static void UpdateCurveTrace(uint16_t range)
//{
//  fMoveP[0][X_Drct] = fMoveP[1][X_Drct];
//  fMoveP[1][X_Drct] = fMoveP[2][X_Drct];
//  fMoveP[0][Y_Drct] = fMoveP[1][Y_Drct];
//  fMoveP[1][Y_Drct] = fMoveP[2][Y_Drct];
//  
//  fMoveP[2][X_Drct] = s16_CalP[0][X_Drct]*MATCH_RATIO*(-t*t*t+3*t*t-3*t+1)+ \
//                      s16_CalP[1][X_Drct]*MATCH_RATIO*(3*t*t*t-6*t*t+4)+ \
//                      s16_CalP[2][X_Drct]*MATCH_RATIO*(-3*t*t*t+3*t*t+3*t+1)+ \
//                      s16_CalP[3][X_Drct]*MATCH_RATIO*t*t*t;
//  
//  fMoveP[2][Y_Drct] = s16_CalP[0][Y_Drct]*MATCH_RATIO*(-t*t*t+3*t*t-3*t+1)+ \
//                      s16_CalP[1][Y_Drct]*MATCH_RATIO*(3*t*t*t-6*t*t+4)+ \
//                      s16_CalP[2][Y_Drct]*MATCH_RATIO*(-3*t*t*t+3*t*t+3*t+1)+ \
//                      s16_CalP[3][Y_Drct]*MATCH_RATIO*t*t*t;
//  
//  t+=0.0043f;//0.0025f;
//  
//  if(t>=1.0f)
//  {
//    CreateNewPoint(range);
//  }
//}


/**
  * @brief  根据坐标计算车移动的两个方向的速度
  * @param  float* point  移动的目标点
  * @retval None
  */
static void CalVxVy(float* point)
{
  g_tBaseUnderpan.fSpeedX = tPID_VX.f_cal_pid(&tPID_VX, *(point+0), tBaseMixLocData.s16_xMix);
  g_tBaseUnderpan.fSpeedY = tPID_VY.f_cal_pid(&tPID_VY, *(point+1), tBaseMixLocData.s16_yMix);
  g_tBaseUnderpan.fOmega  = tPID_Angle.f_cal_pid(&tPID_Angle, g_tBaseUnderpan.fTarAngle, tBaseMixLocData.fAngleMix);
}


///**
//  * @brief  沿着随机生成点(贝塞尔曲线)跑
//  * @param  uint16_t range(单位：mm)
//  * @retval None
//  */
//static void MoveAlongTracePoint(uint16_t range)
//{
//  UpdateCurveTrace(range);
//  CalVxVy(fMoveP[2]);
//}


//float fAdvanceV = 3.6f;
/**
  * @brief  排斥模式
  * @param  None
  * @retval None
  */
//uint16_t Circle2BaseLocLen, Origin2BaseLocLen, UAV2BaseLocLen;
//float fResultantCircle2Base, fResultantUAV2Base, fResultantAll2Base;
//float fComponentF_Circle2Base[2], fComponentF_UAV2Base[2], fComponentF_All2Base[2];

//#define CIRCLE_RADIUS   800

//static void RepellentMode(void)
//{
////  uint16_t Circle2BaseLocLen, Origin2BaseLocLen, UAV2BaseLocLen;
//  //合力大小
////  float fResultantCircle2Base, fResultantUAV2Base, fResultantAll2Base;
//  //分解力
////  float fComponentF_Circle2Base[2], fComponentF_UAV2Base[2], fComponentF_All2Base[2];
//  static int16_t testThing = 0;
//  tUAV_Data.s16_UAV_AbsoluX = tRC_Data.ch1;
//  tUAV_Data.s16_UAV_AbsoluY = tRC_Data.ch2;
//  
//  if(testThing == 0)
//  {
//    testThing ++;
//    fMoveP[1][X_Drct] = tBaseMixLocData.s16_xMix;
//    fMoveP[1][Y_Drct] = tBaseMixLocData.s16_yMix;
//    fMoveP[2][X_Drct] = tBaseMixLocData.s16_xMix;
//    fMoveP[2][Y_Drct] = tBaseMixLocData.s16_yMix;
//  }

//  tUAV_Data.s16_UAV_RelaX = tUAV_Data.s16_UAV_AbsoluX-tBaseMixLocData.s16_xMix;
//  tUAV_Data.s16_UAV_RelaY = tUAV_Data.s16_UAV_AbsoluY-tBaseMixLocData.s16_yMix;
///******************************************************************************/
//  
////  tBaseMixLocData.s16_xMix = fMoveP[1][X_Drct];
////  tBaseMixLocData.s16_yMix = fMoveP[1][Y_Drct];
//  
//  fMoveP[1][X_Drct] = fMoveP[2][X_Drct];
//  fMoveP[1][Y_Drct] = fMoveP[2][Y_Drct];
//  
////  fMoveP[1][X_Drct] = tBaseMixLocData.s16_xMix;
////  fMoveP[1][Y_Drct] = tBaseMixLocData.s16_yMix;
//  
////  Origin2BaseLocLen = GET_HPTNS((float)tBaseMixLocData.s16_xMix, (float)tBaseMixLocData.s16_yMix, 0.0f, 0.0f)+0.001f;
//  Origin2BaseLocLen = GET_HPTNS((float)fMoveP[1][X_Drct], (float)fMoveP[1][Y_Drct], 0.0f, 0.0f)+0.001f;
//  UAV2BaseLocLen = GET_HPTNS((float)tUAV_Data.s16_UAV_RelaX, (float)tUAV_Data.s16_UAV_RelaY, 0.0f, 0.0f)+0.001f;
//  
//  if(Origin2BaseLocLen < CIRCLE_RADIUS)
//  {
//    Circle2BaseLocLen = CIRCLE_RADIUS - Origin2BaseLocLen;
//    
//    //一次反比
////    fResultantCircle2Base = 100.0f/(float)Circle2BaseLocLen;
////    fResultantUAV2Base  = 500.0f/(float)UAV2BaseLocLen;
//    
//    //离无人机最好足够远，离壁可以很近，所以两者系数有50倍的关系
//    fResultantCircle2Base = 20000.0f/((float)Circle2BaseLocLen*(float)Circle2BaseLocLen);
//    fResultantUAV2Base  = 1000000.0f/((float)UAV2BaseLocLen*(float)UAV2BaseLocLen);
//    //假想圆外壁对基地的XY分力
////    fComponentF_Circle2Base[X_Drct] = -tBaseMixLocData.s16_xMix/(float)Origin2BaseLocLen*fResultantCircle2Base;
////    fComponentF_Circle2Base[Y_Drct] = -tBaseMixLocData.s16_yMix/(float)Origin2BaseLocLen*fResultantCircle2Base;
//    fComponentF_Circle2Base[X_Drct] = -fMoveP[1][X_Drct]/(float)Origin2BaseLocLen*fResultantCircle2Base;
//    fComponentF_Circle2Base[Y_Drct] = -fMoveP[1][Y_Drct]/(float)Origin2BaseLocLen*fResultantCircle2Base;

//    //无人机对基地的XY分力
//    fComponentF_UAV2Base[X_Drct] = -tUAV_Data.s16_UAV_RelaX/(float)UAV2BaseLocLen*fResultantUAV2Base;
//    fComponentF_UAV2Base[Y_Drct] = -tUAV_Data.s16_UAV_RelaY/(float)UAV2BaseLocLen*fResultantUAV2Base;
//    //合力对基地的XY分力
//    fComponentF_All2Base[X_Drct] = fComponentF_Circle2Base[X_Drct]+ fComponentF_UAV2Base[X_Drct];
//    fComponentF_All2Base[Y_Drct] = fComponentF_Circle2Base[Y_Drct]+ fComponentF_UAV2Base[Y_Drct];
//    
//    //归一化处理后算得移动方向，计算得到移动到的位置
//    fResultantAll2Base = GET_HPTNS(fComponentF_All2Base[X_Drct], fComponentF_All2Base[Y_Drct], 0, 0)+0.00001f;
//    fMoveP[2][X_Drct] = fMoveP[1][X_Drct] + fComponentF_All2Base[X_Drct]/fResultantAll2Base*fAdvanceV;
//    fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct] + fComponentF_All2Base[Y_Drct]/fResultantAll2Base*fAdvanceV;
//  }
//  else
//  {
//    fMoveP[2][X_Drct] = 0;
//    fMoveP[2][Y_Drct] = 0;
//  }
//  
//  CalVxVy(fMoveP[2]);
//}


/**
  * @brief  圆圈（无人机调试用）
  * @param  r 半径
  * @retval None
  */
static void CircleRun(int16_t r)
{
  static float sfAngle = 0;
  static uint16_t ss16_Period = 2500;
  static int16_t ss16_Num = 0;
  
  fMoveP[2][X_Drct] = cos(sfAngle)*r;
  fMoveP[2][Y_Drct] = sin(sfAngle)*r;
  
  if(tRC_Data.ch2 > 100)
  {
    if(ss16_Period>1000)
      ss16_Period--;
  }
  else if(tRC_Data.ch2 < -100)
  {
    if(ss16_Period<5000)
      ss16_Period++;
  }
  
  ss16_Num = ss16_Num%ss16_Period;
  ss16_Num++;
  sfAngle = (float)ss16_Num/((float)ss16_Period/2)*PI;
}


/**
  * @brief  实时血量改变处理
  * @param  None
  * @retval None
  */
uint32_t u32_JudgeTimRcrd = 0;
uint32_t u32_BldChngInOneSec = 0;
void RealBloodChangedHandler(void)
{
  switch(tJData.realBloodChangedData.way)
  {
    case 0x0:
      switch(tJData.realBloodChangedData.weakId)
      {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03:
          if(tJData.realBloodChangedData.value == 500)//受到大英雄攻击
          {
            tBaseAM.u8_EnemyHeroAppr = 1;
            tBaseAM.tAtckRcrd[HERO_RCRD].u32_TimRcrd = HAL_GetTick();
          }
          else//应该是被小子弹打了
          {
            tBaseAM.u8_EnemyInfAppr = 1;
            tBaseAM.tAtckRcrd[INF_RCRD].u32_TimRcrd = HAL_GetTick();
            tBaseAM.tAtckRcrd[INF_RCRD].s8_WeakId = tJData.realBloodChangedData.weakId;
            tBaseAM.tAtckRcrd[INF_RCRD].fAngRcrd = tBaseMixLocData.fAngleMix;
          }
          break;
        case 0x04:
        case 0x05:
          tBaseAM.u8_EnemyUAVAppr = 1;
          tBaseAM.tAtckRcrd[UAV_RCRD].u32_TimRcrd = HAL_GetTick();
          break;
      }
      break;
    case 0x1:
    case 0x2:
    case 0x3:
    case 0x4:
    case 0x6:
    case 0xa:
      break;
  }
  
  //是否为同一秒
  if(tJData.gameInfo.remainTime != u32_JudgeTimRcrd)
  {
    u32_BldChngInOneSec = 0;
    u32_JudgeTimRcrd = tJData.gameInfo.remainTime;
  }
  
  //计算血量
  switch(tJData.realBloodChangedData.weakId)
  {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03:
      u32_BldChngInOneSec += tJData.realBloodChangedData.value;
      break;
    default:
      break;
  }
  
  //判断是否多辆车
  if(u32_BldChngInOneSec >= 400)
  {
    tBaseAM.u8_TooManyEnemy = 1;
    tBaseAM.tAtckRcrd[MANY_RCRD].u32_TimRcrd = HAL_GetTick();
  }
}


/**
  * @brief  边转边跑
  * @param  tranSpd：平移分量速度
  * @param  rttSpd：旋转分量速度（只在完全乱转有用）
  * @param  Mode：0.完全乱转；1.固定方向扫描
  * @retval None
  */
#define CrtRandNumOnBase(base, range) (base+range*(HAL_RNG_GetRandomNumber(&hrng)-2147483648.f)/2147483648.f)
#define ANGLE_CHANGEPROTECT   75
float fDancingAng = 45;       //不考虑自转，假想他任然在平移(本来就是在平移啊)的角度
float fDancingAngBase = 45;   //zhuangbi的平移角度，只是以这个状态移到不会触发的地方
float fDancingRadian;         //弧度
int16_t s16_Gyro360 = 0;
int8_t s8_RoughlyDrct;        //大致朝向:0为右前；1为右后；2为左后；3为左前
int16_t s16_ScnRttDrct = 1;   //扫描旋转方向
//int8_t s8_CollideWall = -1;
static void RunLikeDancing(int16_t tranSpd, int16_t rttSpd, uint8_t Mode)
{
  static uint8_t  u8_KS_LastSymbol = 10;     //上一次检测到的标志
  static int8_t   s8_Drc = 1;               //旋转方向
  static int8_t   s8_PrtctSymbol = 0;       //是否处于检测保护
  static float    fAngRcrdCollideWall;      //撞墙之后改变平移方向角度记录
  static int16_t  s16_AngChange;            //角度变化后再变化多少角度后再次开启角度检测保护，避免同一面墙被多次误检测
  
  /*
  改变运动的条件：
  1.得到的标志是否改变
  2.是否处于状态变化的检测保护中（在保护中如果检测到标志变为0 则会忽略此次变化）
  */
  if((u8_KS_LastSymbol != u8_KS_Symbol) && (s8_PrtctSymbol == 0 || u8_KS_Symbol != 0))
  {
    s16_Gyro360 = ((int32_t)tBaseMixLocData.fAngleMix)%360;
    s16_Gyro360 = (s16_Gyro360<0)?(s16_Gyro360+360):s16_Gyro360;
    s8_RoughlyDrct = s16_Gyro360/90;
    fAngRcrdCollideWall = tBaseMixLocData.fAngleMix;
    
    //先全变，之后再在什么都没有检测到那里改为没有进保护
    s8_PrtctSymbol = 1;
    
    switch(u8_KS_Symbol)
    {
      case (1<<KEYENCE_FR):
        fDancingAngBase = (540 - s8_RoughlyDrct*90)%360;
        fDancingAng = fDancingAngBase;
        break;
      case (1<<KEYENCE_RB):
        fDancingAngBase = (450 - s8_RoughlyDrct*90)%360;
        fDancingAng = fDancingAngBase;
        break;
      case (1<<KEYENCE_BL):
        fDancingAngBase = (360 - s8_RoughlyDrct*90)%360;
        fDancingAng = fDancingAngBase;
        break;
      case (1<<KEYENCE_LF):
        fDancingAngBase = (270- s8_RoughlyDrct*90)%360;
        fDancingAng = fDancingAngBase;
        break;
      
      //下面考虑一个角的情况
      case (1<<KEYENCE_FR|1<<KEYENCE_RB):
        fDancingAngBase = (495 - s8_RoughlyDrct*90)%360;
        fDancingAng = fDancingAngBase;
        break;
      case (1<<KEYENCE_RB|1<<KEYENCE_BL):
        fDancingAngBase = (405 - s8_RoughlyDrct*90)%360;
        fDancingAng = fDancingAngBase;
        break;
      case (1<<KEYENCE_BL|1<<KEYENCE_LF):
        fDancingAngBase = (315 - s8_RoughlyDrct*90)%360;
        fDancingAng = fDancingAngBase;
        break;
      case (1<<KEYENCE_LF|1<<KEYENCE_FR):
        fDancingAngBase = (585 - s8_RoughlyDrct*90)%360;
        fDancingAng = fDancingAngBase;
        break;
      
      //下面只考虑一个角的情况（应该很少很少，我也不知道有没有，先写了再说）
      case (1<<KEYENCE_LF|1<<KEYENCE_FR|1<<KEYENCE_RB):
      case (1<<KEYENCE_FR|1<<KEYENCE_RB|1<<KEYENCE_BL):
      case (1<<KEYENCE_RB|1<<KEYENCE_BL|1<<KEYENCE_LF):
      case (1<<KEYENCE_BL|1<<KEYENCE_LF|1<<KEYENCE_FR):
        s8_PrtctSymbol = 0;
        break;
      
      case 0:
        s8_PrtctSymbol = 0;
        fDancingAng = CrtRandNumOnBase(fDancingAngBase,45);
        s8_Drc = -s8_Drc;
        break;
      
      default:
        s8_PrtctSymbol = 0;
        break;
    }
    u8_KS_LastSymbol = u8_KS_Symbol;
    fDancingRadian = Ang2Rad(fDancingAng);
  }
  else if(s8_PrtctSymbol == 1)
  {
    s16_AngChange = tBaseMixLocData.fAngleMix - fAngRcrdCollideWall;
    
    if(xABS(s16_AngChange) > ANGLE_CHANGEPROTECT)
    {
      s8_PrtctSymbol = 0;
    }
  }
  g_tBaseUnderpan.fSpeedX = tranSpd * cos(fDancingRadian);
  g_tBaseUnderpan.fSpeedY = tranSpd * sin(fDancingRadian);
  
  if(Mode == 1)
  {
    if(tBaseMixLocData.fAngleMix>=60)
      s16_ScnRttDrct = -1;
    else if(tBaseMixLocData.fAngleMix<=-125)
      s16_ScnRttDrct = 1;
    
    g_tBaseUnderpan.fTarAngle = tBaseMixLocData.fAngleMix + 36.0f * s16_ScnRttDrct;
    g_tBaseUnderpan.fOmega = tPID_Angle.f_cal_pid(&tPID_Angle, g_tBaseUnderpan.fTarAngle, tBaseMixLocData.fAngleMix);;
  }
  else
    g_tBaseUnderpan.fOmega = s8_Drc * rttSpd;
  g_tBaseUnderpan.fTarDrct = tBaseMixLocData.fAngleMix;
}




/**
  * @brief  全自动
  * @param  None
  * @retval None
  */
//不耐烦等待比赛开始时间 单位S;
#define IMPATIENT_TIMOUT 40
uint32_t u32_StartTimRcrd;      //开始超时倒计时时间记录
int32_t s32_ImpatientWaiting = IMPATIENT_TIMOUT;  //超时等待剩余时间
uint32_t u32_InsJudgeCnt = 0;   //指令判断时间，防止丢有用的角度
uint32_t u32_AngStbCntDwn = 0;  //角度稳定计数
extern SW_StatusTypeDef emStart_CSW;

//test
uint32_t testEnterInsCnt = 0;
static void AllAutoMove(void)
{
  //判断比赛开始信号（收到比赛开始信号）
  if((tGameStart.s8_StartFlg == 1 || tGameStart.s8_TimoutFlg == 1) && tGameStart.s8_EndFlg != 1)
  {//陀螺仪数据是否正常
    if(tBaseGyroData.tSelfCheck.dataState == DATANORMAL)
    {//裁判系统数据是否正常 & 剩余血量是否大于2500
      if(tJData.tSelfCheck.dataState == DATANORMAL && tJData.gameInfo.remainLifeValue>=2500)
      {
        if(tBaseAM.u8_TooManyEnemy == 1)
        {
          if(HAL_GetTick() - tBaseAM.tAtckRcrd[MANY_RCRD].u32_TimRcrd > 20000)
          {
            tBaseAM.u8_TooManyEnemy = 0;
          }
          RunLikeDancing(500,600,0);
        }
        //英雄是否出现
        else if(tBaseAM.u8_EnemyHeroAppr == 1)
        {
          if(HAL_GetTick() - tBaseAM.tAtckRcrd[HERO_RCRD].u32_TimRcrd > 20000)
          {
            tBaseAM.u8_EnemyHeroAppr = 0;
          }
          RunLikeDancing(500,600,0);
        }
        else
        {
          //无人机是否出现
          if(tBaseAM.u8_EnemyUAVAppr == 1)
          {
            if(HAL_GetTick() - tBaseAM.tAtckRcrd[UAV_RCRD].u32_TimRcrd > 25000)
            {
              tBaseAM.u8_EnemyUAVAppr = 0;
            }
            RunLikeDancing(550,550,0);
          }
          else
          {
            if(tUpperData.tSelfCheck.dataState == DATANORMAL)
            {
              switch(tUpperData.cmd)
              {
                case STAY_CALM:
                case OBEY_INS:
                  u32_InsJudgeCnt = 200;
                  if((u8_KS_Symbol&(1<<KEYENCE_FR))||(u8_KS_Symbol&(1<<KEYENCE_LF)))
                  {
                    g_tBaseUnderpan.fSpeedX = 0;
                    g_tBaseUnderpan.fSpeedY = 0;
                    g_tBaseUnderpan.fTarAngle = tUpperData.ang;
                    g_tBaseUnderpan.fTarDrct = 0;
                    g_tBaseUnderpan.fOmega  = tPID_Angle.f_cal_pid(&tPID_Angle, g_tBaseUnderpan.fTarAngle, tBaseMixLocData.fAngleMix);
                  }
                  else
                  {
                    g_tBaseUnderpan.fTarAngle = tUpperData.ang;
                    g_tBaseUnderpan.fTarDrct = 0;
                    g_tBaseUnderpan.fOmega  = tPID_Angle.f_cal_pid(&tPID_Angle, g_tBaseUnderpan.fTarAngle, tBaseMixLocData.fAngleMix);
                    g_tBaseUnderpan.fSpeedX = 0;
                    if(xABS(g_tBaseUnderpan.fTarAngle-tBaseMixLocData.fAngleMix) <4)
                    {
                      g_tBaseUnderpan.fSpeedY = 350;
                      u32_AngStbCntDwn++;
                    }
                    else
                    {
                      g_tBaseUnderpan.fSpeedY = 0;
                    }
                  }
                  break;
                case BE_CRAZY:
                  if(u32_InsJudgeCnt>0)
                  {
                    u32_InsJudgeCnt--;
                    g_tBaseUnderpan.fSpeedX = 0;
                    g_tBaseUnderpan.fSpeedY = 0;
                    g_tBaseUnderpan.fTarAngle = tUpperData.ang;
                    g_tBaseUnderpan.fTarDrct = 0;
                    g_tBaseUnderpan.fOmega  = tPID_Angle.f_cal_pid(&tPID_Angle, g_tBaseUnderpan.fTarAngle, tBaseMixLocData.fAngleMix);
                  }
                  else
                  {
                    if(tBaseAM.u8_EnemyInfAppr == 1)
                    {
                      //目标角度。。。
                      switch(tBaseAM.tAtckRcrd[INF_RCRD].s8_WeakId)
                      {
                        case 0:
                          g_tBaseUnderpan.fTarAngle = tBaseAM.tAtckRcrd[INF_RCRD].fAngRcrd+135;
                          break;
                        case 1:
                          g_tBaseUnderpan.fTarAngle = tBaseAM.tAtckRcrd[INF_RCRD].fAngRcrd+55;
                          break;
                        case 2:
                          g_tBaseUnderpan.fTarAngle = tBaseAM.tAtckRcrd[INF_RCRD].fAngRcrd-55;
                          break;
                        case 3:
                          g_tBaseUnderpan.fTarAngle = tBaseAM.tAtckRcrd[INF_RCRD].fAngRcrd-135;
                          break;
                      }
                      if((HAL_GetTick() - tBaseAM.tAtckRcrd[INF_RCRD].u32_TimRcrd>5000) || \
                        (xABS(tBaseMixLocData.fAngleMix-g_tBaseUnderpan.fTarAngle)<4))
                      {
                        tBaseAM.u8_EnemyInfAppr = 0;
                      }
                      else
                      {
                        g_tBaseUnderpan.fSpeedX = 0;
                        g_tBaseUnderpan.fSpeedY = 0;
                        g_tBaseUnderpan.fOmega  = tPID_Angle.f_cal_pid(&tPID_Angle, g_tBaseUnderpan.fTarAngle, tBaseMixLocData.fAngleMix);
                      }
                    }
                    else  //来回扫描
                    {
                      RunLikeDancing(500,0,1);
                    }
                  }
                  break;
                case CANT_STOP:
                  RunLikeDancing(550,550,0);
                  break;
              }
            }
            else
            {
              RunLikeDancing(550,550,0);
            }
          }
        }
      }
      else
      {
        RunLikeDancing(550,550,0);
      }
    }
    else
    {
      RandomSpin();
    }
  }
  else if(tGameStart.s8_EndFlg != 1)//(未收到比赛开始信号 且比赛未结束)
  {
    //开关是否为开始倒计时 (只有在裁判系统离线才开始倒计时)
    if((emStart_CSW == SW_LOOSEN) && (tJData.tSelfCheck.dataState != DATANORMAL))
    {
      if(s32_ImpatientWaiting <= 0)
      {
        tGameStart.s8_TimoutFlg = 1;
      }
      else
      {
        s32_ImpatientWaiting = IMPATIENT_TIMOUT - ((HAL_GetTick()-u32_StartTimRcrd)/1000);
      }
    }
    else if(emStart_CSW == SW_PRESS || tJData.tSelfCheck.dataState == DATANORMAL)
    {
      u32_StartTimRcrd = HAL_GetTick();
    }
    g_tBaseUnderpan.fSpeedX = 0;
    g_tBaseUnderpan.fSpeedY = 0;
    g_tBaseUnderpan.fOmega  = 0;
  }
  else
  {
    g_tBaseUnderpan.fSpeedX = 0;
    g_tBaseUnderpan.fSpeedY = 0;
    g_tBaseUnderpan.fOmega  = 0;
  }
}


/**
  * @brief  随机自旋,屌逼陀螺仪跪了要用，讲道理球用没有
  * @param  None
  * @retval None
  */
static void RandomSpin(void)
{
  static uint16_t u16_timCnt1 = 0, u16_timCnt2 = 0;
  static int8_t  u8_Dir = 1;
  g_tBaseUnderpan.fSpeedX = 0;
  g_tBaseUnderpan.fSpeedY = 0;
  
  
  if(u16_timCnt1>(1000/UNDERPAN_PERIOD))
  {
    g_tBaseUnderpan.fOmega = u8_Dir*CreateRandNum(250.0f, 600.0f);
    u16_timCnt1 = 0;
  }
  else
  {
    u16_timCnt1++;
  }
  
  if(u16_timCnt2>(6000/UNDERPAN_PERIOD))
  {
    u8_Dir = (HAL_RNG_GetRandomNumber(&hrng)>2147483648)?1:-1;
    u16_timCnt2 = 0;
  }
  else
  {
    u16_timCnt2++;
  }
}




/**
  * @brief  线段运动，改变MoveP的值影响轨迹，一定时间随机改变方向
  * @param  x值较小的那个点
  * @param  x值较大的那个点
  * @retval None
  */
//#define UINT_LEN    3     //单位前进长度
//#define C_DRCT_TIM  900   //改变方向的时间(首先看能不能整除)
//float fK, fB, fAng, fRad;
//float fDx = 0, fDy = 0;

//static void MoveInLineSegment(float* pfXmin, float* pfXmax)
//{
////  static float fK, fB, fAng, fRad;
//  static int8_t s8_Drct = 1;
//  static ArmAtckMoveModeTypedef tLastLineMode;
//  static uint16_t u16_TimCnt = 0;
////  float fDx, fDy;
//  if(tBaseAM.emAtackMoveMode != MOVE_NO_ATTACK)
//  {
//    fDx = *(pfXmax+X_Drct) - *(pfXmin+X_Drct);
//    fDy = *(pfXmax+Y_Drct) - *(pfXmin+Y_Drct);
//      
//    //随机改变方向时间
//    if(u16_TimCnt<C_DRCT_TIM/UNDERPAN_PERIOD)
//    {
//      u16_TimCnt++;
//    }
//    else
//    {
//      s8_Drct = RNG_GETNUM()>0.5f?-1:1;
//      u16_TimCnt = 0;
//    }
//  
//    if(tLastLineMode != tBaseAM.emAtackMoveMode)
//    {
//      fMoveP[2][X_Drct] = CreateRandNum(*(pfXmin+X_Drct),fDx);
//      if(fDx!=0)
//      {
//        fK = fDy/fDx;
//        fB = *(pfXmax+Y_Drct) -fK*(*(pfXmax+X_Drct));
//        fMoveP[2][Y_Drct] = fK*fMoveP[2][X_Drct]+fB;
//      }
//      else
//      {
//        fMoveP[2][Y_Drct] = CreateRandNum(*(pfXmin+Y_Drct),fDy);
//      }
//    }
//    else
//    {
//      fRad = atan2(fDy, fDx);
//      fMoveP[2][X_Drct] = fMoveP[1][X_Drct]+s8_Drct*cos(fRad)*UINT_LEN;
//      fMoveP[2][Y_Drct] = fMoveP[1][Y_Drct]+s8_Drct*sin(fRad)*UINT_LEN;
//      if(fMoveP[2][Y_Drct]>((*(pfXmin+Y_Drct))>(*(pfXmax+Y_Drct))?(*(pfXmin+Y_Drct)):(*(pfXmax+Y_Drct))) || \
//        fMoveP[2][Y_Drct]<((*(pfXmin+Y_Drct))<(*(pfXmax+Y_Drct))?(*(pfXmin+Y_Drct)):(*(pfXmax+Y_Drct))) || \
//        fMoveP[2][X_Drct]>*(pfXmax+X_Drct)||fMoveP[2][X_Drct]<*(pfXmin+X_Drct))
//      {
//        s8_Drct = -s8_Drct;
//        fMoveP[2][X_Drct] += s8_Drct*cos(fRad)*UINT_LEN*2;
//        fMoveP[2][Y_Drct] += s8_Drct*sin(fRad)*UINT_LEN*2;
//      }
//      
//    }
//    fMoveP[1][X_Drct] = fMoveP[2][X_Drct];
//    fMoveP[1][Y_Drct] = fMoveP[2][Y_Drct];
//  }
//  tLastLineMode = tBaseAM.emAtackMoveMode;
//}


/**
  * @brief  固定直线轨迹跑，根据装甲板受到伤害的情况
  * @param  AtckMngTypeDef AtckMng      //讲道理这个被优化了。。。球用没有，和指针没有区
  * @param  AtckMngTypeDef* p_tBaseAM
  * @param  uint16_t u16_Range
  * @retval None
  */
//static void ArmourInjuredRun(AtckMngTypeDef* p_tBaseAM, uint16_t u16_Range)
//{
//  int16_t s16_Cnt = 0;             //求和计数
//  float fMaxNum = 0;

//  //4.5秒判断是否受到攻击
//  if(HAL_GetTick() - p_tBaseAM->tAtckRcrd[p_tBaseAM->u16_AtckRcrdCnt].u32_TimRcrd >4500)
//  {
//    p_tBaseAM->u8_EffFlg = 0; 
//    tBaseAM.emAtackMoveMode = MOVE_NO_ATTACK;
//  }
//  else
//  {
//    p_tBaseAM->u8_EffFlg = 1; 
//  }
//  //至少被打中了啊
//  if(1==p_tBaseAM->u8_EffFlg && p_tBaseAM->u16_EffRcrdCnt >= 1)
//  {
//    //总量清零,各个百分比总量清零
//    p_tBaseAM->u16_WeakBldSum = 0;
//    p_tBaseAM->fAvrWeakId = 0;
//    p_tBaseAM->fEachIdWeakPercent[ARMOUR_F] = 0;
//    p_tBaseAM->fEachIdWeakPercent[ARMOUR_L] = 0;
//    p_tBaseAM->fEachIdWeakPercent[ARMOUR_B] = 0;
//    p_tBaseAM->fEachIdWeakPercent[ARMOUR_R] = 0;
//    
//    //扣血总量求和,各个装甲受到总量求和
//    for(int16_t i=0;i<p_tBaseAM->u16_EffRcrdCnt;i++)
//    {
//      s16_Cnt = p_tBaseAM->u16_AtckRcrdCnt - i;
//      if(s16_Cnt < 0)
//      {
//        s16_Cnt += ATTACKRCRD_NUM;
//      }
//      p_tBaseAM->u16_WeakBldSum += p_tBaseAM->tAtckRcrd[s16_Cnt].u16_Val;
//    }
//    
//    //清零
//    s16_Cnt = 0;
//    //平均扣血ID,各个ID权重计数
//    for(int16_t i=0;i<p_tBaseAM->u16_EffRcrdCnt;i++)
//    {
//      s16_Cnt = p_tBaseAM->u16_AtckRcrdCnt - i;
//      if(s16_Cnt < 0)
//      {
//        s16_Cnt += ATTACKRCRD_NUM;
//      }
//      p_tBaseAM->fAvrWeakId += p_tBaseAM->tAtckRcrd[s16_Cnt].u16_Val / \
//            ((float)p_tBaseAM->u16_WeakBldSum) * p_tBaseAM->tAtckRcrd[s16_Cnt].s8_WeakId;
//      p_tBaseAM->fEachIdWeakPercent[p_tBaseAM->tAtckRcrd[s16_Cnt].s8_WeakId] += p_tBaseAM->tAtckRcrd[s16_Cnt].u16_Val / \
//            (float)p_tBaseAM->u16_WeakBldSum;
//    }        
//    //求出最大权重ID
//    p_tBaseAM->s8_MaxWeakId = -1;
//    for(int16_t i=0;i<4;i++)
//    {
//      if(p_tBaseAM->fEachIdWeakPercent[i] > fMaxNum)
//      {
//        fMaxNum = p_tBaseAM->fEachIdWeakPercent[i];
//        p_tBaseAM->s8_MaxWeakId = i;
//      }
//    }
//    p_tBaseAM->u16_WeakBldSum = p_tBaseAM->u16_WeakBldSum;
//    p_tBaseAM->fAvrWeakId = p_tBaseAM->fAvrWeakId;
//    p_tBaseAM->s8_MaxWeakId = p_tBaseAM->s8_MaxWeakId;
//    
//    p_tBaseAM->fEachIdWeakPercent[0] = p_tBaseAM->fEachIdWeakPercent[0];
//    p_tBaseAM->fEachIdWeakPercent[1] = p_tBaseAM->fEachIdWeakPercent[1];
//    p_tBaseAM->fEachIdWeakPercent[2] = p_tBaseAM->fEachIdWeakPercent[2];
//    p_tBaseAM->fEachIdWeakPercent[3] = p_tBaseAM->fEachIdWeakPercent[3];
//    
//    //恢复默认排序
//    p_tBaseAM->s8_WeakIdSort[0] = 0;
//    p_tBaseAM->s8_WeakIdSort[1] = 1;
//    p_tBaseAM->s8_WeakIdSort[2] = 2;
//    p_tBaseAM->s8_WeakIdSort[3] = 3;
//    
//    float fTmp;
//    //冒泡排序,从0到3，从大到小
//    for(int16_t i=4;i>0;i--)
//    {
//      for(uint8_t j=1;j<i;j++)
//      {
//        if(p_tBaseAM->fEachIdWeakPercent[j-1] < p_tBaseAM->fEachIdWeakPercent[j])
//        {
//          fTmp = p_tBaseAM->fEachIdWeakPercent[j];
//          p_tBaseAM->fEachIdWeakPercent[j] = p_tBaseAM->fEachIdWeakPercent[j-1];
//          p_tBaseAM->fEachIdWeakPercent[j-1] = fTmp;
//          fTmp = p_tBaseAM->s8_WeakIdSort[j];
//          p_tBaseAM->s8_WeakIdSort[j] = p_tBaseAM->s8_WeakIdSort[j-1];
//          p_tBaseAM->s8_WeakIdSort[j-1] = (int8_t)fTmp;
//        }
//      }
//    }
//    
//    //选出因装甲板受到伤害要跑动的方式
//    switch(p_tBaseAM->s8_WeakIdSort[0])
//    {
//      case ARMOUR_F:
//        if(p_tBaseAM->fEachIdWeakPercent[p_tBaseAM->s8_WeakIdSort[0]] - \
//          p_tBaseAM->fEachIdWeakPercent[p_tBaseAM->s8_WeakIdSort[1]]<=0.1f)
//        {
//          switch(p_tBaseAM->s8_WeakIdSort[1])
//          {
//            case ARMOUR_L:
//              p_tBaseAM->emAtackMoveMode = MOVE_FL_ATTACK;
//              break;
//            case ARMOUR_R:
//              p_tBaseAM->emAtackMoveMode = MOVE_FR_ATTACK;
//              break;
//          }
//        }
//        else
//        {
//          p_tBaseAM->emAtackMoveMode = MOVE_F_ATTACK;
//        }
//        break;
//      case ARMOUR_L:
//        p_tBaseAM->emAtackMoveMode = MOVE_FL_ATTACK;
//        break;
//      case ARMOUR_R:
//        p_tBaseAM->emAtackMoveMode = MOVE_FR_ATTACK;
//        break;
//      default:
//        break;
//    }
//    switch(p_tBaseAM->emAtackMoveMode)
//    {
//      case MOVE_F_ATTACK:
//        fXminPoint[X_Drct] = -u16_Range;
//        fXminPoint[Y_Drct] = u16_Range;
//        fXmaxPoint[X_Drct] = u16_Range;
//        fXmaxPoint[Y_Drct] = -u16_Range;
//        break;
//      case MOVE_FL_ATTACK:
//        fXminPoint[X_Drct] = -u16_Range;
//        fXminPoint[Y_Drct] = -u16_Range;
//        fXmaxPoint[X_Drct] = u16_Range;
//        fXmaxPoint[Y_Drct] = -u16_Range;
//        break;
//      case MOVE_FR_ATTACK:
//        fXminPoint[X_Drct] = -u16_Range;
//        fXminPoint[Y_Drct] = -u16_Range;
//        fXmaxPoint[X_Drct] = -u16_Range;
//        fXmaxPoint[Y_Drct] = u16_Range;
//        break;
//      default:
//        break;
//    }
//  }
//  MoveInLineSegment(fXminPoint, fXmaxPoint); //可能我只是进去更新一下tLastLineMode
//}


/**
  * @brief  限制功率速度输出，电流分配
  * @param  输入x,y方向速度,角速度omega，目标角度值
  * @retval None
  */
static void CalWheelSpdUnderPowerManage(float vx, float vy, float omega, float tarAngle)
{
  CalculateWheelSpeed(vx, vy, omega, Ang2Rad(tarAngle), AUTO_WHEEL_SPD_MAX); //按最大移动速度
  
  //计算实际使用总电流值
  g_tPowerManage.s16_AllWheelCur = tMotorData[W_Right].s16_RealCur + \
                                  tMotorData[W_Front].s16_RealCur + \
                                  tMotorData[W_Left].s16_RealCur + \
                                  tMotorData[W_Behind].s16_RealCur;
  //计算剩余电流
  g_tPowerManage.s16_ResidueCur = g_tPowerManage.s16_CurLimit - g_tPowerManage.fAllPower;
  //计算所需电流值
  int16_t s16_AllNeededCur = tMotorData[W_Right].s16_NeededCur + \
                            tMotorData[W_Front].s16_NeededCur + \
                            tMotorData[W_Left].s16_NeededCur + \
                            tMotorData[W_Behind].s16_NeededCur;
  
  if(0 == s16_AllNeededCur)
  {
    g_tPowerManage.s16_GivenMaxCur[W_Right] = g_tPowerManage.s16_CurLimit/4;
    g_tPowerManage.s16_GivenMaxCur[W_Front] = g_tPowerManage.s16_CurLimit/4;
    g_tPowerManage.s16_GivenMaxCur[W_Left] = g_tPowerManage.s16_CurLimit/4;
    g_tPowerManage.s16_GivenMaxCur[W_Behind] = g_tPowerManage.s16_CurLimit/4;
  }
  else
  {
    g_tPowerManage.s16_GivenMaxCur[W_Right] = (int16_t)((float)tMotorData[W_Right].s16_NeededCur* \
                                                  (float)g_tPowerManage.s16_CurLimit/ \
                                                  (float)s16_AllNeededCur);
    g_tPowerManage.s16_GivenMaxCur[W_Front] = (int16_t)((float)tMotorData[W_Front].s16_NeededCur* \
                                                  (float)g_tPowerManage.s16_CurLimit/ \
                                                  (float)s16_AllNeededCur);
    g_tPowerManage.s16_GivenMaxCur[W_Left] = (int16_t)((float)tMotorData[W_Left].s16_NeededCur* \
                                                  (float)g_tPowerManage.s16_CurLimit/ \
                                                  (float)s16_AllNeededCur);
    g_tPowerManage.s16_GivenMaxCur[W_Behind] = (int16_t)((float)tMotorData[W_Behind].s16_NeededCur* \
                                                  (float)g_tPowerManage.s16_CurLimit/ \
                                                  (float)s16_AllNeededCur);
  }
  
  //发送到can队列
  sendMessageToCANQueue(CAN_FourMotor_ID, g_tBaseUnderpan.s16_WheelSpd);
  sendMessageToCANQueue(CAN_FourCur_ID, g_tPowerManage.s16_GivenMaxCur);
}


/**
  * @brief  调试PID模式
  * @param  None
  * @retval None
  */
//static void MoveDebugPID(void)
//{
//}


/**
  * @brief  复位陀螺仪
  * @param  None
  * @retval None
  */
//static void ResetGyro(void)
//{
//  sendMessageToCANQueue(CAN_GyroReset_ID, (void*)0);
//}


/**
  * @brief  复位雷达
  * @param  None
  * @retval None
  */
//static void ResetRadar(void)
//{
//  sendMessageToCANQueue(CAN_RadarReset_ID, (void*)0);
//}

/**
  * @brief  底盘PID配置
  * @param  None
  * @retval None
  */
static void Underpan_PID_Configuration(void)
{
  PID_Configuration(
    &tPID_Angle,  //	PID_TypeDef* pid,
    PID_ANGLE,    //	PID_ID   id,
    2000,        //	uint16_t maxout,
    65535,        //	uint16_t intergral_limit,
    1,            //	uint16_t deadband,
    10,           //	uint16_t period,
    0,            //	int16_t  max_err,
    0,            //	float    target,

    10,           //	float 	kp,
    0,            //	float 	ki,
    100,          //	float 	kd
    DB_OUTPUT_ZERO);  //死区输出选择
  
  PID_Configuration(
    &tPID_VX,     //	PID_TypeDef* pid,
    PID_VX,       //	PID_ID   id,
    2000,         //	uint16_t maxout,
    65535,        //	uint16_t intergral_limit,
    0,            //	uint16_t deadband,
    10,           //	uint16_t period,
    0,            //	int16_t  max_err,
    0,            //	float    target,

    2.5,            //	float 	kp,
    0,            //	float 	ki,
    21,            //	float 	kd
    DB_OUTPUT_LAST);  //死区输出选择
    
  PID_Configuration(
    &tPID_VY,     //	PID_TypeDef* pid,
    PID_VY,       //	PID_ID   id,
    2000,         //	uint16_t maxout,
    65535,        //	uint16_t intergral_limit,
    0,            //	uint16_t deadband,
    10,           //	uint16_t period,
    0,            //	int16_t  max_err,
    0,            //	float    target,

    2.5,            //	float 	kp,
    0,            //	float 	ki,
    21,            //	float 	kd
    DB_OUTPUT_LAST);  //死区输出选择
}


/**
  * @brief  Create the UnderpanTask threads
  * @param  osPriority taskPriority
  * @retval None
  */
void underpanTaskThreadCreate(osPriority taskPriority)
{
	osThreadDef(underpanTask, UnderpanTask, taskPriority, 0, 512);
  underpanTaskHandle = osThreadCreate(osThread(underpanTask), NULL);
}
