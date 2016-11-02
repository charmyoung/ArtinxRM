#ifndef __CAN2_H__
#define __CAN2_H__
#include "stdint.h"


void CAN2_Configuration(void);

//RM3510
void Cmd_ESC_820R(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);


#endif 
