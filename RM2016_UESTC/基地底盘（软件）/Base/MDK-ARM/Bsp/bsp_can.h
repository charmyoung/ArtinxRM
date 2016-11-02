#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "can.h"
#include "includes.h"


#define CHANGE_MIAXDATASRC2(n)    do{tBaseMixLocData.emMixSrc = n;}while(0)


/** 
  * @brief  CAN发送或是接收的ID
  */
typedef enum
{
  CAN_6623CTRL_ID   = 0x1FF,  //6623控制ID
  CAN_24V_YAW_ID    = 0x205,  //6623YAW轴ID
  CAN_24V_PITCH_ID  = 0x206,  //6623Pitch轴ID
	CAN_FourMotor_ID  = 0x046,	//四轮
	CAN_GyroRecAg_ID	= 0x011,	//陀螺仪接收角度
	CAN_GyroRecXY_ID	=	0x012,	//陀螺仪接收x,y坐标
  CAN_GyroReset_ID  = 0x013,	//陀螺仪复位
  CAN_RadarReset_ID = 0x014,  //雷达复位
  CAN_Radar_ID      = 0x058,  //雷达接收
  CAN_UAV_ID        = 0x02B,  //飞机位置
  CAN_MotorR_ID     = 0x041,  //右侧电机
  CAN_MotorF_ID     = 0x042,  //前方电机
  CAN_MotorL_ID     = 0x043,  //左侧电机
  CAN_MotorB_ID     = 0x044,  //后方电机
  CAN_FourCur_ID    = 0x040,  //发送给四个轮子的四个电流
  CAN_KDetect_ID    = 0x015,  //基恩士检测到的状态的ID
  CAN_UPPER_ID      = 0x303,  //云台部分发过来的ID
}CAN_Message_ID;


/** 
  * @brief 数据转换联合体
  */
typedef union
{
 uint8_t  u8_form[4];
 int32_t  s32_form;
 float    float_form;
}DataConvertTypeDef;


/** 
  * @brief 雷达状态枚举
  */
typedef enum
{
  HAVENOTRECEIVE  = 0x00, //未曾收到
  INITiALIZING    = 0x01, //初始化状态
  BELIEVABLE      = 0x02, //可信的
  UNBELIEVABLE    = 0x03, //不可信的
}RadarStatusTypedef;


/** 
  * @brief 雷达数据结构体
  */
typedef struct
{
  RadarStatusTypedef emRadarStatus;
  uint8_t u8_DataType;
  int8_t  s8_RcvAngle;
  int16_t s16_RcvX;
  int16_t s16_RcvY;
  int16_t s16_RcvO;
  int8_t  s8_RadarAngle;
  int16_t s16_RadarX;
  int16_t s16_RadarY;
  SelfCheckTypeDef  tSelfCheck;
}RadarDataTypeDef;


/** 
  * @brief 陀螺仪数据结构体
  */
typedef struct
{
  float fInitAngle;           //初始角度
  float fGyroAngle;           //接收到的陀螺仪角度值
  float fGyroAngleTmp;
  float fGyroMapanX;
  float fGyroMapanY;
  float fGyroMapanX_cnt;
  float fGyroMapanY_cnt;
  int32_t s32_EncX_cnt;       //接收到的X方向编码器值
  int32_t s32_EncY_cnt;       //接收到的Y方向编码器值
  int32_t s32_EncDx_cnt;
  int32_t s32_EncDy_cnt;
  SelfCheckTypeDef  tSelfCheck;
}GyroDataTypeDef;


/** 
  * @brief 融合坐标数据来源选择枚举
  */
typedef enum
{
  MAPAN_ONLY = 0x00,
  RADAR_ONLY = 0x01,
  M_AND_R    = 0x02,
}DataSrcChooseTypeDef;

/** 
  * @brief 融合坐标数据结构体
  */
typedef struct
{
  #define ANGLE_CT_LEN 10
  DataSrcChooseTypeDef  emMixSrc;
  float   fAngleMix;
  float   fRadianMix;   //弧度值
  struct
  {
    float   fAngleErr[ANGLE_CT_LEN];
    uint8_t AngleErrCT;
  }AngleErrRcd;         //角度数据记录
  float   fAngleErrEf;  //有效角度误差
  int16_t s16_xMix;
  int16_t s16_yMix;
  int16_t s16_xErr;
  int16_t s16_yErr;
  int16_t s16_xErrEf;   //x有效误差
  int16_t s16_yErrEf;   //y有效误差
}MixLocDataTypeDef;


/** 
  * @brief  无人机位置状态
  */
typedef enum
{
  UAV_LOST    = 1,
  UAV_FOUND   = 2,
}UAV_StateTypeDef;


/** 
  * @brief 无人机状态及数据
  */
typedef struct
{
  int16_t   s16_UAV_RelaX;        //相对于车的X坐标
  int16_t   s16_UAV_RelaY;        //相对于车的Y坐标
  int16_t   s16_UAV_AbsoluX;      //绝对的X坐标
  int16_t   s16_UAV_AbsoluY;      //绝对的Y坐标
  UAV_StateTypeDef  em_UAV_State; //无人机状态
  SelfCheckTypeDef  tSelfCheck;
}UAV_DataTypeDef;


/** 
  * @brief  轮子位置
  */
enum
{
  W_Right   = 0,
  W_Front   = 1,
  W_Left    = 2,
  W_Behind  = 3,
};


/** 
  * @brief  雷达传回数据信息
  */
enum
{
  ONE_ANG = 11,
  TWO_ANG = 12,
  THR_ANG = 13,
  BREAK_P = 2,      //有断点（许学姐说很少出现,然而明明有很多）
  JUNK_DATA = 0,    //废物数据（不用处理）
};


/** 
  * @brief  底盘电机状态数据结构。。。纪念姚学长。。。
  */
typedef struct
{
  int16_t s16_NeededCur;    //所需电流值
  int16_t s16_RevSpd;       //轮子转速
  int16_t s16_RealCur;      //实际电流值
#pragma pack(push)
#pragma pack(1)
	union{
		struct{
			unsigned can:1;
			unsigned encoder:1;
			unsigned adc:1;
			unsigned usart:1;
			unsigned :4;
		}bit;
		uint8_t BYTE;
	}ESR;
#pragma pack(pop)
  SelfCheckTypeDef  tSelfCheck;
}MotorDataTypeDef;


/** 
  * @brief  云台电机数据结构体
  */
typedef struct
{
  uint8_t rcvFlag;              //是否收到过电机信息
  uint16_t u16_LastAngleTmp;    //上一次收到的角度
  uint16_t u16_AngleTmp;        //这一次收到的角度
  int32_t s32_AngleBase;        //基准值
  int32_t s32_AngleAfCal;       //计算后的角度
  int32_t s32_AngleAfCalLast;   //上一次计算的角度
  int16_t s16_OmgAfCal;         //计算得到的角速度
  int16_t s16_RealCurrent;      //实际电流值
  int16_t s16_GivenCurrent;     //给定电流值
  uint16_t u16_CurBydCnt;       //电流超范围计数
  uint8_t  u8_CurBydFlg;        //电流超范围了，应该是卡蛋
  uint16_t u16_CurBydFlgCnt;    //电流超了计数
  SelfCheckTypeDef  tSelfCheck;
}GimbalMeasureTypeDef;


/** 
  * @brief  云台电机检测can是否在线结构体
  */
typedef struct
{
  SelfCheckTypeDef  tSelfCheck;
}GimbalCheckTypeDef;

/** 
  * @brief  云台部分数据
  */
typedef enum
{
  STAY_CALM = 0,  //静止不动
  BE_CRAZY  = 1,  //乱动
  OBEY_INS  = 2,  //服从指令
  CANT_STOP = 3,  //根本停不下来
}upperCmdemTypeDef;

/** 
  * @brief  云台部分数据
  */
typedef struct
{
  upperCmdemTypeDef cmd;
  float ang;
  SelfCheckTypeDef  tSelfCheck;
}UpperDataTyperDef;



void My_CAN_FilterConfig(CAN_HandleTypeDef* _hcan);
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, CAN_Message_ID, int16_t* _message, uint8_t* _pBuff);

#endif
