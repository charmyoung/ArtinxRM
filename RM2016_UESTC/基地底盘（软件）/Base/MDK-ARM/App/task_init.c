/**
  ******************************************************************************
  * @file    task_init.c
  * @author  EC_Dog
  * @version V1.0
  * @date    2016/06/01
  * @brief   
  * 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_oled.h"
#include "junru_libs.h"
#include "judgement_libs.h"
#include "task_init.h"
#include "task_underpan.h"
#include "task_check.h"
#include "task_debug.h"
#include "timer_canSender.h"
#include "task_bulletLoading.h"
#include "task_keyPress.h"
#include "task_oled.h"
#include "task_detect.h"
/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
osThreadId initTaskHandle;

extern uint8_t arrRC_Buf[];
extern uint8_t judgementBuf[];
extern uint8_t arrBT_RxBuf[];


void InitTask(void const * argument)
{
  //摩擦轮电调初始化
  HAL_TIM_PWM_Start(&FWHEEL_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&FWHEEL_TIM, TIM_CHANNEL_2);
	FWHEEL_TIM.Instance->CCR1=1000;
	FWHEEL_TIM.Instance->CCR2=1000;
  //气缸初始化
  HAL_GPIO_WritePin(BASE_JJ,GPIO_PIN_SET);

  osDelay(500);
  //can接收中断
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  //遥控器串口空闲中断
  Junru_UART_Receive_IT(&DBUS_HUART, arrRC_Buf, RC_BUF_LEN);
  //裁判系统串口空闲中断
  Junru_UART_Receive_IT(&JUDGE_HUART, judgementBuf, JUDGEMENT_BUFLEN);
  //蓝牙串口空闲中断
  Junru_UART_Receive_IT(&BLUET_HUART, arrBT_RxBuf, BT_RXBUF_LEN);
  
  //创建底盘任务
  underpanTaskThreadCreate(osPriorityAboveNormal);
//  //创建补弹任务
//  bulletLoadingThreadCreate(osPriorityAboveNormal);
  //创建自检任务
  checkTaskThreadCreate(osPriorityBelowNormal);
  //创建调试任务
  debugTaskThreadCreate(osPriorityNormal);
  //创建按键检测
  keyPressTaskThreadCreate(osPriorityBelowNormal);
  //can定时器发送
  canSenderTimerCreate();
  //基恩士检测任务
  detectTaskThreadCreate(osPriorityAboveNormal);
  //oled任务
  oledTaskThreadCreate(osPriorityLow);

  
  for(;;)
  {
    vTaskDelete(initTaskHandle);
  }
}


/**
  * @brief  Create the UnderpanTask threads
  * @param  None
  * @retval None
  */
void initTaskThreadCreate(osPriority taskPriority)
{
	osThreadDef(initTask, InitTask, taskPriority, 0, 256);
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);
}
