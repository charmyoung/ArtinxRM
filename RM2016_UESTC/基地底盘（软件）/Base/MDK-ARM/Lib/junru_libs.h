#ifndef __JUNRU_LIBS_H
#define __JUNRU_LIBS_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef Junru_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef Junru_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void Junru_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size);

#endif
