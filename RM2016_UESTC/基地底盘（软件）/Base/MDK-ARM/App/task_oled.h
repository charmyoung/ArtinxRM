#ifndef __TASK_OLED_H
#define __TASK_OLED_H

#include "cmsis_os.h"


enum
{
  FR = 0,
  RB,
  BL,
  LF,
};

enum
{
  Red = 0,
  Blu,
};

void oledTaskThreadCreate(osPriority taskPriority);


#endif
