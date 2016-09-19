#ifndef __USART3_H__
#define __USART3_H__
#include "stdint.h"

#define USART_REC_LEN 6
extern unsigned char USART_RX_BUF[USART_REC_LEN];
void USART3_Configuration(void);
void USART3_SendChar(unsigned char b);
void USART3_SendStr(uint8_t *str);
void USART3_TX_ROS(void);

#endif
