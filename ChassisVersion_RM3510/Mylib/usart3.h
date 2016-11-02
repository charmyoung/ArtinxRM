#ifndef __USART3_H__
#define __USART3_H__
#include "stdint.h"

#define USART_REC_LEN 6
extern unsigned char USART_RX_BUF[USART_REC_LEN];
void USART3_Configuration(void);


#endif
