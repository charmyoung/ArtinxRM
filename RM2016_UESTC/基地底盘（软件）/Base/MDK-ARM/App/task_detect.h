#ifndef __TASK_DETECT_H
#define __TASK_DETECT_H

#include "stm32f4xx.h"
#include "cmsis_os.h"

/** 
  * @brief  KEYENCE Location enum definition,极限8个
  */
enum
{
  KEYENCE_F   = 0,    //前
  KEYENCE_FR,         //前右
  KEYENCE_R,          //右
  KEYENCE_RB,         //右后
  KEYENCE_B,          //后
  KEYENCE_BL,         //后左
  KEYENCE_L,          //左
  KEYENCE_LF,         //左前
  KEYENCE_LIST_LEN,
};

/** 
  * @brief  KEYENCE State enum definition
  */
typedef enum
{
  NOT_DETECTED  = 0,
  MAY_DETECTED  = 1,
  DETECTED      = 2,
}KEYENCE_StateTypeDef;

/** 
  * @brief  KEYENCE State structures definition
  */
typedef struct
{
  GPIO_TypeDef* tpK_IO;
  uint32_t  K_PIN;
  uint16_t  u16_VerifyCnt;
  KEYENCE_StateTypeDef emK_State;
}KEYENCE_DataTypeDef;


void detectTaskThreadCreate(osPriority taskPriority);


#endif
