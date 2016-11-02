//这些东西我实在是不知道要放在哪里比较好

#ifndef __INCLUDES_H
#define __INCLUDES_H


#ifndef UESTC__ONE_POINT_FIVE_S__NO_1
  #error "Cofidence!!!"
#endif

#if defined DEBUG_MODE
  #warning "You are debuging!!!"
#endif


#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define PI        3.141592653589793f
//绝对值
#define xABS(m)  ((m)>=0?(m):-(m))
//角度&弧度 相互转化
#define Ang2Rad(m)  (m/180.0f*PI)
#define Rad2Ang(n)  (n/PI*180.f)

#define UPDATE_CHECKTIME(n) do{ \
  (n)->tSelfCheck.lastUpdateTime = (n)->tSelfCheck.recentUpdateTime; \
  (n)->tSelfCheck.recentUpdateTime = HAL_GetTick(); \
  (n)->tSelfCheck.upDateTimeLen = (n)->tSelfCheck.recentUpdateTime - (n)->tSelfCheck.lastUpdateTime; \
  }while(0)


/** 
  * @brief  Data State enum definition
  */
typedef enum
{
  OFFLINE = 0x00,   //设备离线
  RESETING,         //设备复位
  DATANORMAL,       //数据值正常
  DATAEXCEPTION,    //数据值异常
  DATATIMEOUT,      //更新时间超长
}DataStateTypeDef;


/** 
  * @brief  Selfcheck structures definition
  */
typedef struct
{
  TickType_t        lastUpdateTime;     //上一次更新时间
  TickType_t        recentUpdateTime;   //最近更新时间
  TickType_t        upDateTimeLen;      //更新时长
  DataStateTypeDef  dataState;          //数据状态
}SelfCheckTypeDef;

#endif
