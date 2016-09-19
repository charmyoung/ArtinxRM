#ifndef __CAN1_H__
#define __CAN1_H__
#include "stdint.h"

void CAN1_Configuration(void);
void CurrentProtect(void);
void Cmd_ESC(int16_t current_205,int16_t current_206);
void Cmd_Reset(void);

#endif
