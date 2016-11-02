#ifndef __TIMER_CANSENDER_H
#define __TIMER_CANSENDER_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "bsp_can.h"

/** 
  * @brief  CAN_Message_TypeDef  struct
  */
typedef struct
{
  CAN_Message_ID  emCAN_ID;
  int16_t         s16_Data[4];
}CAN_Message_TypeDef;


BaseType_t sendMessageToCANQueue(CAN_Message_ID _id, int16_t* _message);
void canSenderTimerCreate(void);


#endif
