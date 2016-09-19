#include "stm32f4xx.h"

void NVIC_Configuration(void);
void NVIC_Set(int Channel,int PreemptionPriority,int SubPriority,FunctionalState Cmd);
