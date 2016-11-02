#ifndef __TASK_CHECK_H
#define __TASK_CHECK_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


/** @defgroup Tone_Height_selection Tone Height selection
  * @brief    Tone Height selection,自动重装载值（TIM：84MHz；Prescaler：167）
  * @{
  */ 
#define Do_L /*261.63Hz*/    382
#define Re_L /*293.66Hz*/    341
#define Mi_L /*329.63Hz*/    303
#define Fa_L /*349.23Hz*/    286
#define So_L /*392.00Hz*/    255
#define La_L /*440.00Hz*/    227
#define Si_L /*493.88Hz*/    202

#define Do_M /*523.25Hz*/    191
#define Re_M /*587.33Hz*/    170
#define Mi_M /*659.26Hz*/    152
#define Fa_M /*698.46Hz*/    143
#define So_M /*784.00Hz*/    128
#define La_M /*880.00Hz*/    114
#define Si_M /*987.77Hz*/    101

#define Do_H /*1046.50Hz*/   96
#define Re_H /*1174.66Hz*/   85
#define Mi_H /*1318.51Hz*/   76
#define Fa_H /*1396.91Hz*/   72
#define So_H /*1567.98Hz*/   64
#define La_H /*1760.00Hz*/   57
#define Si_H /*1975.53Hz*/   51
/**
* @}
*/


enum
{
  S_note    = 1,   //sixteenth note  十六分音符
  E_note    = 2,   //eighth note 
  DEnote    = 3,   //Dotted eighth note
  Q_note    = 4,   //quarter note
  DQnote    = 6,   //Dotted quarter note
  H_note    = 8,   //half note
  DHnote    = 12,  //Dotted half note
  W_Note    = 16,  //whole note
};


/** 
  * @brief  自检数据项
  */
enum
{
  GYRO_DATACHECK     = 0,
  RADAR_DATACHECK,
  UAV_DATACHECK,
  MOTOR_L_DATACHECK,
  MOTOR_R_DATACHECK,
  MOTOR_F_DATACHECK,
  MOTOR_B_DATACHECK,
  JUDGE_DATACHECK,
  RC_DATACHECK,
  PIT_DATACHECK,  //云台电机pitch轴
  YAW_DATACHECK,  //云台电机yaw轴
  UPPER_DATACHECK,//上面发下来的
  DATACHECK_LEN,
};


///** 
//  * @brief  鸣叫结构体(存16个数据)
//  */
//typedef struct
//{
//  uint8_t 
//}


void checkTaskThreadCreate(osPriority taskPriority);


#endif
