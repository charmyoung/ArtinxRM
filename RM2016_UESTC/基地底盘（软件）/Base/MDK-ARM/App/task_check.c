/**
  ******************************************************************************
  * @file    task_check.c
  * @author  EC_Dog
  * @version V2.0
  * @date    2016/05/26
  * @brief   
  * 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "task_check.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "judgement_libs.h"
#include "tim.h"
/* Defines -------------------------------------------------------------------*/
#define BEEP_HTIM     htim8
#define BEEP_CHANNEL  TIM_CHANNEL_3
#define BEEP_CCR      CCR3

#define CHECK_PERIOD 50
/* Variables -----------------------------------------------------------------*/
osThreadId checkTaskHandle;

//换成数组写法 音符数组[(代表不同八度][(代表不同调)]
uint16_t u16_DownByTheSG[][2]={
{Do_M,E_note},{Re_M,E_note},{Mi_M,Q_note},{Re_M,E_note},
{Do_M,E_note},{Re_M,Q_note},{Mi_M,E_note},{So_M,E_note},
{La_M,H_note},{So_M,Q_note},{Do_H,E_note},{So_M,E_note},
{La_M,Q_note},{So_M,E_note},{Mi_M,E_note},{Re_M,Q_note},
{Do_M,E_note},{Re_M,E_note},{Mi_M,H_note},{Mi_M,Q_note},
};

SelfCheckTypeDef* selfCheckList[DATACHECK_LEN] = {NULL};


extern RC_TypeDef           tRC_Data;
extern GyroDataTypeDef      tBaseGyroData;
extern RadarDataTypeDef     tBaseRadarData;
extern UAV_DataTypeDef      tUAV_Data;
extern MotorDataTypeDef     tMotorData[4];
extern JudgementDataTypedef tJData;
extern GimbalCheckTypeDef   tGimbalPitch;
extern GimbalCheckTypeDef   tGimbalYaw;
extern UpperDataTyperDef    tUpperData;


static void beepSet(uint16_t Prd, float fVol);
static void beepBiBi(uint8_t counts, uint16_t time_ms);
static void checkRxDatas(void);
//static void dataStateOutput(void);

/**
  * @brief  sing task
  * @param  void const * argument
  * @retval None
  */
void CheckTask(void const * argument)
{
  beepSet(200, 0.03);    //500Hz
  selfCheckList[GYRO_DATACHECK] = &(tBaseGyroData.tSelfCheck);
//  selfCheckList[RADAR_DATACHECK] = &(tBaseRadarData.tSelfCheck);
//  selfCheckList[UAV_DATACHECK] = &(tUAV_Data.tSelfCheck);
  selfCheckList[MOTOR_L_DATACHECK] = &(tMotorData[W_Left].tSelfCheck);
  selfCheckList[MOTOR_R_DATACHECK] = &(tMotorData[W_Right].tSelfCheck);
  selfCheckList[MOTOR_F_DATACHECK] = &(tMotorData[W_Front].tSelfCheck);
  selfCheckList[MOTOR_B_DATACHECK] = &(tMotorData[W_Behind].tSelfCheck);
  selfCheckList[JUDGE_DATACHECK] = &(tJData.tSelfCheck);
  selfCheckList[PIT_DATACHECK] = &(tGimbalPitch.tSelfCheck);
  selfCheckList[YAW_DATACHECK] = &(tGimbalYaw.tSelfCheck);
  selfCheckList[UPPER_DATACHECK] = &(tUpperData.tSelfCheck);
//  selfCheckList[RC_DATACHECK] = &(tRC_Data.tSelfCheck);
  
  //延时开始检测
//  osDelay(2500);
//  for(int i=0;i<sizeof(u16_DownByTheSG)/4;i++)
//  {
//    HAL_TIM_PWM_Start(&BEEP_HTIM, BEEP_CHANNEL);
//    BEEP_HTIM.Instance->ARR = u16_DownByTheSG[i][0];
//    osDelay(u16_DownByTheSG[i][1]*50);
//  }
//  HAL_TIM_PWM_Stop(&BEEP_HTIM, BEEP_CHANNEL);
  uint16_t cnt = 0;
  for(;;)
  {
    osDelay(CHECK_PERIOD);
    checkRxDatas();
    
    if(NULL != selfCheckList[cnt])
    {
      switch(selfCheckList[cnt]->dataState)
      {
        case DATANORMAL:
        case RESETING:
          break;
        
        case OFFLINE:
        case DATAEXCEPTION:
        case DATATIMEOUT:
          beepBiBi(cnt<5?1:(cnt<10?2:3), 600);
          beepBiBi(cnt%5+1, 600);
          osDelay(150);
          break;
      }
    }
    cnt = (cnt+1)%DATACHECK_LEN;
    osDelay(CHECK_PERIOD);
  }
}


/**
  * @brief  蜂鸣器设置鸣叫如有问题请检测配置
  * @param  uint16_t Prd 周期相关
  * @param  float fVol  音量（0-1）,这是一个二次函数
  * @retval None
  */
static void beepSet(uint16_t Prd, float fVol)
{
  BEEP_HTIM.Instance->ARR = Prd - 1;
  BEEP_HTIM.Instance->BEEP_CCR = Prd*fVol;
}


/**
  * @brief  蜂鸣器鸣叫，鸣叫间隔时间为100ms
  * @param  uint8_t counts 鸣叫次数
  * @param  uint16_t time  鸣叫总时间(ms)
  * @retval None
  */
static void beepBiBi(uint8_t counts, uint16_t time_ms)
{
  for(uint8_t i=counts;i>0;i--)
  {
    HAL_TIM_PWM_Start(&BEEP_HTIM, BEEP_CHANNEL);
    osDelay(time_ms/counts-100);
    HAL_TIM_PWM_Stop(&BEEP_HTIM, BEEP_CHANNEL);
    osDelay(100);
  }
  osDelay(100);
}


///**
//  * @brief  用一个十六位数据移位定义鸣叫
//  * @param  uint8_t trigType 触发哔哔哔类型
//  * @param  uint16_t beepType 鸣叫类型
//  * @retval None
//  */
//static void beepBiBiBi(uint8_t trigType, uint16_t beepType)
//{
//  
//}


/**
  * @brief  接收数据处理
  * @param  None
  * @retval None
  */
static void checkRxDatas(void)
{
  uint32_t timeNow = HAL_GetTick();
  int32_t timeDiff;
  for(uint8_t i=0;i<DATACHECK_LEN;i++)
  {
    //判断地址是否为空
    if(NULL != selfCheckList[i])
    {
      timeDiff = timeNow - selfCheckList[i]->recentUpdateTime;
      switch(selfCheckList[i]->dataState)
      {
        case DATANORMAL:
        case DATAEXCEPTION:
          if(timeDiff > 400) {
            selfCheckList[i]->dataState = DATATIMEOUT;
          }else {
            selfCheckList[i]->dataState = DATANORMAL;
          }
          break;
          
        case DATATIMEOUT:
        case OFFLINE:
          if(timeDiff > 400) {
            selfCheckList[i]->dataState = OFFLINE;
          }else {
            selfCheckList[i]->dataState = DATANORMAL;
          }
          break;
        
        //这个写的有问题,跳不出reseting
        case RESETING:
          if(timeDiff > 2500) {
            selfCheckList[i]->dataState = DATATIMEOUT;
          }else {
//            selfCheckList[i]->dataState = RESETING;
            selfCheckList[i]->dataState = DATATIMEOUT;
          }
          break;
      }
    }
  }
}


/**
  * @brief  数据状态响应输出选择鸣叫
  * @param  None
  * @retval None
  */
//static void dataStateOutput(void)
//{
//  for(uint8_t i=0;i<DATACHECK_LEN;i++)
//  {
//    //判断地址是否为空
//    if(NULL != selfCheckList[i])
//    {
//      switch(selfCheckList[i]->dataState)
//      {
//        case DATANORMAL:
//        case RESETING:
//          break;
//        
//        case OFFLINE:
//        case DATAEXCEPTION:
//        case DATATIMEOUT:
//          beepBiBi(i<5?1:2, 600);
//          beepBiBi(i%5+1, 600);
//          osDelay(200);
//          break;
//      }
//    }
//  }
//}


/**
  * @brief  Create the Check Task threads
  * @param  None
  * @retval None
  */
void checkTaskThreadCreate(osPriority taskPriority)
{
	osThreadDef(checkTask, CheckTask, taskPriority, 0, 128);
  checkTaskHandle = osThreadCreate(osThread(checkTask), NULL);
}

