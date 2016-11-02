#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "includes.h"

#define DBUS_HUART  huart2
#define JUDGE_HUART huart4
#define BLUET_HUART huart1

#define RC_BUF_LEN    100
#define BT_RXBUF_LEN  20
#define BT_TXBUF_LEN  20

/** 
  * @brief  遥控数据结构体
  */
typedef struct{
	//遥控器通道
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	uint8_t switch_left;
	uint8_t switch_right;
	//鼠标
	struct{
		int16_t x;
		int16_t y;
		int16_t z;
		
		uint16_t press_left;
		uint16_t press_right;
	}mouse;
	//键盘
	uint16_t key_code;
	
	uint8_t keyBoardFlag[16][2];
  SelfCheckTypeDef  tSelfCheck;
/*************************************************************************************
   * 键盘通道:15   14   13   12   11   10    9   8    7    6     5    4    3    2    1
   *          V    C    X	   Z    G    F    R    E   Q   CTRL  SHIFT  D    A    S    W
**************************************************************************************/
}RC_TypeDef;


//enum
//{
//	NOW = 0,
//	LAST = 1,
//};


/** 
  * @brief  遥控器上的开关值
  */
enum
{
  No_Data       = 0,
	Switch_Up     = 1,
	Switch_Middle = 3,
	Switch_Down   = 2,
};


/** 
  * @brief  鼠标按键状态
  */
enum
{
	Press_Up = 0,
	Press_Down = 1,
};



void UART_Printf(UART_HandleTypeDef *huart, const char* fmt, ...);


#endif
