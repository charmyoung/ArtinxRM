#ifndef TASK_BULLETLOADING_H
#define TASK_BULLETLOADING_H

#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "tim.h"

#define FWHEEL_TIM    htim2
#define BASE_JJ       GPIOA,GPIO_PIN_7

/** 
  * @brief  补弹任务状态
  */
typedef struct
{
  uint8_t u8_IsLoadingFinish;
  uint8_t u8_Is45AngAdjCmplt;     //初始45度校准完成
  uint8_t u8_OperatingStep;       //操作步骤 
  uint16_t u16_PosRdyCnt;         //位置消抖
}LoadingStatusTypeDef;


/** 
  * @brief  比赛开始信息
  */
typedef struct
{
  int8_t s8_StartCntDwnFlg;
  int8_t s8_StartFlg;
  int8_t s8_TimoutFlg;
  int8_t s8_EndFlg;
}GameStartTypeDef;



void bulletLoadingThreadCreate(osPriority taskPriority);

#endif
