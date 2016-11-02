#ifndef __TASK_KEYPRESS_H
#define __TASK_KEYPRESS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


/** 
  * @brief  ¿ª¹Ø×´Ì¬
  */
typedef enum
{
  SW_LOOSEN     = 0,
  SW_MAYLOOSEN  = 1,
  SW_MAYPRESS   = 2,
  SW_PRESS      = 3,
}SW_StatusTypeDef;


void keyPressTaskThreadCreate(osPriority taskPriority);

#endif
