/**
  ******************************************************************************
  * @file    timer_canSend.c
  * @author  EC_Dog
  * @version V1.0
  * @date    2016/01/21
  * @brief   can的定时器发送,防止抢占
  * 
  ******************************************************************************
  * @attention  内部有一些用于测试的东西！
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timer_canSender.h"
#include "queue.h"
#include "can.h"

/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
osMessageQId canSendQueueHandle;
osTimerId canSendTimerHandle;

CAN_Message_TypeDef messageBuff;

//test variable
int16_t testSpace;
HAL_StatusTypeDef testCanStatus;


/**
  * @brief  CAN timer callback
  * @param  void const * argument
  * @retval None
  */
void canSendTimerCallback(void const * argument)
{
	while(xQueueReceive(canSendQueueHandle, &messageBuff, 1))
	{
  //test(OK!).......................................................................................................
    testSpace = uxQueueSpacesAvailable(canSendQueueHandle);
  //test(OK!).......................................................................................................
		testCanStatus=CAN_Send_Message(&hcan1, messageBuff.emCAN_ID, messageBuff.s16_Data, (void*)0);
	}
//	//防止can错误（cube专用）
//	if((hcan1.State&HAL_CAN_STATE_BUSY_RX)==HAL_CAN_STATE_BUSY_RX && !__HAL_CAN_GET_IT_SOURCE(&hcan1,CAN_IT_FMP0))
//		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
}


/**
  * @brief  添加数据到can队列(把所有添加到can队列的写出一个函数)
  * @param  CAN_Message_ID _id
  * @param  int16_t* _message
  * @retval BaseType_t
  */
BaseType_t sendMessageToCANQueue(CAN_Message_ID _id, int16_t* _message)
{
	BaseType_t queueStatus;
	CAN_Message_TypeDef canMessageBuff;
  canMessageBuff.emCAN_ID = _id;
  
	switch(_id)
	{
		case CAN_FourMotor_ID:
    case CAN_FourCur_ID:
			for(int ii=0;ii<4;ii++)
        canMessageBuff.s16_Data[ii] = *(_message + ii);
    	queueStatus = xQueueSend(canSendQueueHandle, &canMessageBuff, (TickType_t)1);
			break;

		case CAN_GyroReset_ID:
			break;
		
    case CAN_RadarReset_ID:
      break;
    
    case CAN_6623CTRL_ID:
      canMessageBuff.s16_Data[0] = *(_message);
    	queueStatus = xQueueSend(canSendQueueHandle, &canMessageBuff, (TickType_t)1);
      break;
    
		default:
			break;
	}
//	queueStatus = xQueueSend(canSendQueueHandle, &canMessageBuff, (TickType_t)1);
	return(queueStatus);
}


/**
  * @brief  Create the can send queue
  * @param  None
  * @retval None
  */
void CAN_QueueCreate(void)
{
  osMessageQDef(canSendQueue, 20, CAN_Message_TypeDef);
  canSendQueueHandle = osMessageCreate(osMessageQ(canSendQueue), NULL);
}


/**
  * @brief  创建can发送定时器
  * @param  None
  * @retval None
  */
void canSenderTimerCreate(void)
{
  //先创建队列
  CAN_QueueCreate();
  
	osTimerDef(canSendTimer, canSendTimerCallback);
	canSendTimerHandle = osTimerCreate(osTimer(canSendTimer), osTimerPeriodic, NULL);
	
	osTimerStart(canSendTimerHandle, 1);
}
