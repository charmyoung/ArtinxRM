/**
  ******************************************************************************
  * @file    task_bulletLoading.c
  * @author  EC_Dog
  * @version V0.0
  * @date    2016/07/01
  * @brief   
  *       V0.0  第一版补弹机构
  ******************************************************************************
  * @attention
  *     
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "task_bulletLoading.h"
#include "cmsis_os.h"
#include "bsp_pid.h"
#include "bsp_can.h"
#include "task_underpan.h"
#include "timer_canSender.h"
#include "task_keyPress.h"
#include "judgement_libs.h"
/* Defines -------------------------------------------------------------------*/
#define LOAD_PX_2     60            //第二次补弹位置X
#define LOAD_PY_2     -730          //第二次补弹位置Y
#define HERO_LOAD_QUA 8            //第一个补英雄 +1
#define INFT_LOAD_QUA 7             //第一个补步兵 +1
#define FIR_LOAD_QUA  11            //第一次补弹数量
#define SEC_LOADPOSRDY_CNT  1000    //第二次补弹消抖时间

#define ERECT_JJ    HAL_GPIO_WritePin(BASE_JJ,GPIO_PIN_RESET)
#define HANG_JJ     HAL_GPIO_WritePin(BASE_JJ,GPIO_PIN_SET)
#define LUST_UP     do{FWHEEL_TIM.Instance->CCR1=1150; \
                      FWHEEL_TIM.Instance->CCR2=1150; \
                      }while(0);
#define LUST_DOWN   do{FWHEEL_TIM.Instance->CCR1=1000; \
                      FWHEEL_TIM.Instance->CCR2=1000; \
                      }while(0);
/* Variables -----------------------------------------------------------------*/
osThreadId bulletLoadingTaskHandle;

LoadingStatusTypeDef  tBaseLdSt;
GameStartTypeDef      tGameStart; //比赛开始

PID_TypeDef tPID_EggRotateAng;    //转蛋pid位置环
PID_TypeDef tPID_EggRotateOmg;    //转蛋pid速度环

__IO uint16_t u16_AngPrd = 10;	//角度环周期
__IO uint16_t u16_OmgPrd = 5;		//角速度环周期

uint16_t u16_AngAfCalIn8192 = 0;//计算后的角度（0-8191）
int32_t s32_EggAngTar = 0;
uint8_t u8_FirAngIndex = 0;     //第一个角度索引值
uint8_t u8_AngIndex = 0;
                      
uint8_t u8_LoadQuaChose = INFT_LOAD_QUA;

uint8_t u8_AngPrdCnt = 0;
uint8_t u8_OmgPrdCnt = 0;

int16_t LbrLdCircleCnt = 0;       //人工补弹转过的绝对圈数
int16_t CurSend = 0;

int8_t s8_LoadingStuckFlg = 0;    //补弹卡了。。。
//固定移动点
float fFixedMovePoint[][2] = {{0,0},{-500,0},{60,0},{60,0},{-825,-895}};
//固定角度
uint16_t u16_FixedAngle[16] ={
8020, 7505, 6995, 6495, 5955, 5425, 4910, 4390,
3900, 3415, 2940, 2450, 1940, 1410,  870,  350,
};

extern GimbalMeasureTypeDef   tEggMotorMeasure;
extern GyroDataTypeDef        tBaseGyroData;
extern UnderpanDataTypeDef    g_tBaseUnderpan;
extern MixLocDataTypeDef      tBaseMixLocData;
extern JudgementDataTypedef   tJData;
//extern SW_StatusTypeDef       emLoadChose_CSW;          //选择补英雄还是步兵开关
//extern SW_StatusTypeDef       emLbrLdCmplt_TSW;         //人工补弹完成状态改变触发


/*test*************************************************/
int16_t testCur = 0, testCnt = 0, testshoot =0, testup = 0, testSpd = 1150;


uint8_t FindNearbyEncoder(uint16_t CurrentEncoder, uint16_t *StepEncoder);

/**
  * @brief  补弹任务
  * @param  void const * argument
  * @retval None
  */
void BulletLoadingTask(void const * argument)
{
//  portTickType xLastWakeTime;
//  
//  //补弹持续时间
//  int16_t loadingDur = 0;
//  
////  //test
////  int16_t j = 0, i=0;
//  
//  //补弹角度选择
//  uint8_t loadingAngCs = 0;
//  
//  PID_Configuration(
//    &tPID_EggRotateAng,  //	PID_TypeDef* pid,
//    PID_EGGROTATE_ANG,    //	PID_ID   id,
//    6000,        //	uint16_t maxout,
//    30,        //	uint16_t intergral_limit,
//    0,            //	uint16_t deadband,
//    u16_AngPrd,   //	uint16_t period,
//    0,            //	int16_t  max_err,
//    0,            //	float    target,
//    0.095,           //	float 	kp，
//    0,           //	float 	ki，
//    0,          //	float 	kd
//    DB_OUTPUT_LAST);  //死区输出选择

//  PID_Configuration(
//    &tPID_EggRotateOmg,  //	PID_TypeDef* pid,
//    PID_EGGROTATE_OMG,    //	PID_ID   id,
//    6000,        //	uint16_t maxout,
//    1000,        //	uint16_t intergral_limit,
//    0,            //	uint16_t deadband,
//    u16_OmgPrd,   //	uint16_t period,
//    0,            //	int16_t  max_err,
//    0,            //	float    target,
//    -35,           //	float 	kp,
//    0,            //	float 	ki,
//    0,          //	float 	kd
//    DB_OUTPUT_LAST);  //死区输出选择
//  
//  while(tEggMotorMeasure.rcvFlag!=1)
//  {
//    osDelay(1);
//  }
//  
////  u8_FirAngIndex = FindNearbyEncoder((uint16_t)tEggMotorMeasure.s32_AngleAfCal, u16_FixedAngle);
////  s32_EggAngTar = u16_FixedAngle[u8_FirAngIndex];
//  tPID_EggRotateOmg.f_pid_reset(&tPID_EggRotateOmg,0,0,0);
//  
//  xLastWakeTime = xTaskGetTickCount();
//  for(;;)
//  {
//    osDelayUntil(&xLastWakeTime, 1);
//    
//    //test，讲道理灯开始闪了证明这个任务在运行
//    if(testCnt<=500){
//      testCnt++;
//    }else{
//      testCnt = 0;
//      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
//    }
//    //test
//    
//    //比赛开始前可修改，按键触发
//    if(tGameStart.s8_StartFlg != 1)
//    {     
//      if(emLoadChose_CSW == SW_LOOSEN)
//      {
//        u8_LoadQuaChose = INFT_LOAD_QUA;
//      }
//      else
//      {
//        u8_LoadQuaChose = HERO_LOAD_QUA;
//      }
//    }
//    
//    /***************双环调节*****************/
//    u8_AngPrdCnt++;
//    u8_OmgPrdCnt++;
//    if(u8_OmgPrdCnt>=u16_OmgPrd)
//    {
//      tEggMotorMeasure.s16_OmgAfCal = tEggMotorMeasure.s32_AngleAfCal - tEggMotorMeasure.s32_AngleAfCalLast;
//      tEggMotorMeasure.s32_AngleAfCalLast = tEggMotorMeasure.s32_AngleAfCal;
//      tPID_EggRotateOmg.f_cal_pid(&tPID_EggRotateOmg, tPID_EggRotateAng.output, tEggMotorMeasure.s16_OmgAfCal);
//      u8_OmgPrdCnt = 0;
//      CurSend = (int16_t)tPID_EggRotateOmg.output;
//    }

//    if(u8_AngPrdCnt>=u16_AngPrd)
//    {
//      tPID_EggRotateAng.f_cal_pid(&tPID_EggRotateAng, s32_EggAngTar, tEggMotorMeasure.s32_AngleAfCal);
//      u8_AngPrdCnt = 0;
//    }
//    sendMessageToCANQueue(CAN_6623CTRL_ID, &CurSend);
//    /***************双环调节*****************/
//    
//    if(g_tBaseUnderpan.emBaseMoveMode == MoveFullAuto && \
//        tBaseLdSt.u8_OperatingStep<=5 && \
//        tBaseGyroData.tSelfCheck.dataState == DATANORMAL)
//    {
//      switch(tBaseLdSt.u8_OperatingStep)
//      {
//        case 0://初始位置
//          if((xABS(tBaseGyroData.fGyroMapanX-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][X_Drct])<=30) && \
//            (xABS(tBaseGyroData.fGyroMapanY-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][Y_Drct])<=30) && \
//            (emLbrLdCmplt_TSW == SW_LOOSEN||tGameStart.s8_StartFlg == 1))
//          {
//            if(tBaseLdSt.u16_PosRdyCnt>=1000)
//            {
//              //就近找到第一个索引值
//              if((tEggMotorMeasure.s32_AngleAfCal%8192)<0)
//              {
//                u16_AngAfCalIn8192 = tEggMotorMeasure.s32_AngleAfCal%8192+8192;
//                LbrLdCircleCnt = tEggMotorMeasure.s32_AngleAfCal/8192-1;
//              }
//              else
//              {
//                u16_AngAfCalIn8192 = tEggMotorMeasure.s32_AngleAfCal%8192;
//                LbrLdCircleCnt = tEggMotorMeasure.s32_AngleAfCal/8192;
//              }
//              u8_FirAngIndex = FindNearbyEncoder(u16_AngAfCalIn8192, u16_FixedAngle);
//              s32_EggAngTar = u16_FixedAngle[u8_FirAngIndex] + LbrLdCircleCnt*8192;
//              tPID_EggRotateOmg.f_pid_reset(&tPID_EggRotateOmg,-35,0,0);
//              
//              tBaseLdSt.u8_OperatingStep = 1;
//              tBaseLdSt.u16_PosRdyCnt = 0; //清零
//            }
//            else
//            {
//              tBaseLdSt.u16_PosRdyCnt++;
//            }
//          }
//          break;
//        
//        case 1:
//          if((xABS(tBaseGyroData.fGyroMapanX-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][X_Drct])<=30) && \
//            (xABS(tBaseGyroData.fGyroMapanY-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][Y_Drct])<=30))
//          {
//            if(tBaseLdSt.u16_PosRdyCnt>=2000 && \
//              (tBaseLdSt.u16_PosRdyCnt>=31000 || tGameStart.s8_StartFlg == 1)) //31s后还未收到比赛开始信号强制启动
//            {
//              tBaseLdSt.u8_OperatingStep = 2;
//              tBaseLdSt.u16_PosRdyCnt = 0; //清零
//              ERECT_JJ;
//            }
//            else if(tBaseLdSt.u16_PosRdyCnt<31000)
//            {
//              tBaseLdSt.u16_PosRdyCnt++;
//            }
//          }
//          break;
//          
//        case 2://到达初始补弹位置
//          if((xABS(tBaseGyroData.fGyroMapanX-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][X_Drct])<=30) && \
//            (xABS(tBaseGyroData.fGyroMapanY-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][Y_Drct])<=30))
//          {
//            if(tBaseLdSt.u16_PosRdyCnt>=3200)
//            {
//              LUST_UP;
//              tBaseLdSt.u8_OperatingStep = 3;
//              tBaseLdSt.u16_PosRdyCnt = 0; //清零
//              loadingDur = 0;
//            }
//            else
//            {
//              tBaseLdSt.u16_PosRdyCnt++;
//            }
//          }
//          break;
//          
//        case 3://进行补弹操作(...越写越长)
//          if(tEggMotorMeasure.u8_CurBydFlg != 1)
//          { //1.补弹是否超时，2.是否在转换位置并进行第二个补弹位置的第一次射击
//            if(loadingDur<=1900 && \
//              (tBaseLdSt.u16_PosRdyCnt==0||(tBaseLdSt.u16_PosRdyCnt>SEC_LOADPOSRDY_CNT&&loadingAngCs!=u8_LoadQuaChose))){
//              loadingDur++;
//            }else{
//              if(loadingAngCs<15 && s8_LoadingStuckFlg != 1){
//                //第一次补弹
//                if(loadingAngCs<u8_LoadQuaChose)
//                {
//                  loadingAngCs++;
//                  u8_AngIndex = (u8_FirAngIndex+loadingAngCs*3)%16;
//                }
//                else
//                {
//                  //第二次补弹点
//                  fFixedMovePoint[3][0] = LOAD_PX_2;
//                  fFixedMovePoint[3][1] = LOAD_PY_2;
//                  if((xABS(tBaseGyroData.fGyroMapanX-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][X_Drct])<=30) && \
//                    (xABS(tBaseGyroData.fGyroMapanY-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][Y_Drct])<=30))
//                  {
//                    if(tBaseLdSt.u16_PosRdyCnt>SEC_LOADPOSRDY_CNT)
//                    {
//                      loadingAngCs++;
//                      u8_AngIndex = (u8_FirAngIndex+loadingAngCs*3)%16;
//                    }
//                    else
//                    {
//                      tBaseLdSt.u16_PosRdyCnt++;
//                    }
//                  }
//                }
//                s32_EggAngTar = u16_FixedAngle[u8_AngIndex] - \
//                                8192*((u8_FirAngIndex+loadingAngCs*3)/16) + \
//                                LbrLdCircleCnt*8192;
//              }else{
//                tBaseLdSt.u8_OperatingStep = 4;
//                tBaseLdSt.u16_PosRdyCnt = 0;
//              }
//              loadingDur = 0;
//            }
//          }
//          else
//          {
//            loadingDur = 800;
//            if(loadingAngCs>1)
//              loadingAngCs--;
//            u8_AngIndex = (u8_FirAngIndex+loadingAngCs*3)%16;
//            s32_EggAngTar = u16_FixedAngle[u8_AngIndex] - \
//                                8192*((u8_FirAngIndex+loadingAngCs*3)/16) + \
//                                LbrLdCircleCnt*8192;
//            tEggMotorMeasure.u8_CurBydFlg = 0;
//            
//            tEggMotorMeasure.u16_CurBydFlgCnt++;
//            
//            if(tEggMotorMeasure.u16_CurBydFlgCnt == 15)
//            {
//              tBaseLdSt.u8_OperatingStep = 4;
//              tBaseLdSt.u16_PosRdyCnt = 0;
//            }
//          }
//          break;
//        
//        case 4://跑到校准点
//          if((xABS(tBaseGyroData.fGyroMapanX-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][X_Drct])<=30) && \
//            (xABS(tBaseGyroData.fGyroMapanY-fFixedMovePoint[tBaseLdSt.u8_OperatingStep][Y_Drct])<=30))
//          {
//            if(tBaseLdSt.u16_PosRdyCnt>=2000)
//            {
//              tPID_EggRotateOmg.f_pid_reset(&tPID_EggRotateOmg,0,0,0);
//              tBaseLdSt.u8_OperatingStep = 5;
//              tBaseLdSt.u16_PosRdyCnt = 0; //清零
//              HANG_JJ;
//              LUST_DOWN;
//              CHANGE_MIAXDATASRC2(M_AND_R);
//            }
//            else
//            {
//              tBaseLdSt.u16_PosRdyCnt++;
//            }
//          }
//          break;
//          
//        case 5://完成
//          if(tBaseLdSt.u16_PosRdyCnt>=4000)
//          {
//            tBaseLdSt.u8_OperatingStep++;
//            tBaseLdSt.u16_PosRdyCnt = 0; //清零
//            tBaseLdSt.u8_IsLoadingFinish = 1;
//          }
//          else{
//            tBaseLdSt.u16_PosRdyCnt++;
//          }
//          break;
//      }
//    }
//  }
}


/**
  * @brief  就近原则找初始点
  * @param  uint16_t CurrentEncoder, uint16_t *StepEncoder
  * @retval index
  */
uint8_t FindNearbyEncoder(uint16_t CurrentEncoder, uint16_t *StepEncoder)
{
	uint8_t forindex;
	uint8_t NearbyIndex = 0;
	int16_t MinError = xABS((int16_t)CurrentEncoder - StepEncoder[0]);
	
	for(forindex = 1; forindex < 16; forindex++)
	{
		if(xABS((int16_t)CurrentEncoder - StepEncoder[forindex]) < MinError)
		{
			MinError = xABS((int16_t)CurrentEncoder - StepEncoder[forindex]);
			NearbyIndex = forindex;
		}
	}
	
	return NearbyIndex;
}


/**
  * @brief  实时射击信息处理（不用了。。。）
  * @param  None
  * @retval None
  */
int8_t ShootCompleteFlg = 0;
uint32_t lastShootTimRcrd;

void RealShootDataHandler(void)
{
  lastShootTimRcrd = HAL_GetTick();
}


/**
  * @brief  比赛信息处理
  * @param  None
  * @retval None
  */
void GameInfoHandler(void)
{
//  static uint32_t u32_LasttimRecord = 0;
  if(tGameStart.s8_StartCntDwnFlg == 0 && \
        tJData.gameInfo.remainTime != 0)
  {
    tGameStart.s8_StartCntDwnFlg = 1;
  }
  else if(tGameStart.s8_StartCntDwnFlg != 0 && \
        tJData.gameInfo.remainTime <= 420)
  {
    tGameStart.s8_StartFlg = 1;
  }
  else if(tGameStart.s8_StartFlg == 1 && \
        tJData.gameInfo.remainTime >= 540)
  {
    tGameStart.s8_EndFlg = 1;
  }
//  u32_LasttimRecord = tJData.gameInfo.remainTime;
}


/**
  * @brief  Create the bullet loading threads
  * @param  osPriority taskPriority
  * @retval None
  */
void bulletLoadingThreadCreate(osPriority taskPriority)
{
	osThreadDef(bulletLoadingTask, BulletLoadingTask, taskPriority, 0, 128);
  bulletLoadingTaskHandle = osThreadCreate(osThread(bulletLoadingTask), NULL);
}

