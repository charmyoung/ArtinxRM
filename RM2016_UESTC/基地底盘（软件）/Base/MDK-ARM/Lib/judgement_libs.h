#ifndef __JUDGEMENT_LIBS_H
#define __JUDGEMENT_LIBS_H

#include "stm32f4xx_hal.h"
#include "includes.h"

#define JUDGEMENT_BUFLEN 60


/** 
  * @brief  命令码ID
  */
typedef enum{
  GameInfoId              = 0x0001, //比赛进程信息
  RealBloodChangedDataId  = 0x0002, //实时血量变化数据
  RealShootDataId         = 0x0003, //实时射击数据
}comIdType;


/** 
  * @brief  eBuffType enum definition,四个小符点状态
  */
typedef enum{
  BUFF_TYPE_NONE,           //无效
  BUFF_TYPE_ARMOR   = 0x01, //防御符
  BUFF_TYPE_SUPPLY  = 0x04, //加血符
  BUFF_TYPE_BULLFTS = 0x08, //加弹符
}eBuffType;


/** 
  * @brief  FrameHeader structure definition,帧头结构体
  */
typedef __packed struct
{
  uint8_t   sOF;
  uint16_t  dataLenth;
  uint8_t   cRC8;
}tFrameHeader;


/** 
  * @brief  GPS state structures definition,GPS状态结构体
  */
typedef __packed struct
{
  uint8_t flag; //0 无效， 1 有效
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t compass;
}tGpsData;


/** 
  * @brief  Game information structures definition,比赛进程信息（ 0x0001）
  *         此数据包的发送频率为 50Hz
  */
typedef __packed struct
{
  uint32_t remainTime;        /*比赛剩余时间（从倒计时三分钟开始计算，单位 s）*/
  uint16_t remainLifeValue;   /*机器人剩余血量*/
  float realChassisOutV;      /*实时底盘输出电压（单位 V）*/
  float realChassisOutA;      /*实时底盘输出电流（单位 A）*/
  
  eBuffType runeStatus[4];    /*四个小符点状态，可能是：(这波“可能”我服了)
                                0x00 = 无效；
                                0x01 = 防御符；
                                0x04 = 加血符；
                                0x08 = 加弹符*/
  
  uint8_t bigRune0Status;     /*大符点 1 状态：
                                0x00 = 无效；
                                0x01 = 有效，无机器人占领；
                                0x02 = 机器人正在占领；
                                0x03 = 已占领*/
  uint8_t bigRune1status;     /*大符点 2. 其值意义同大符点 1*/
  
  uint8_t conveyorBelts0:2;   /*传送带以及停机坪状态：0： 1bits: 0 号传送带状态：
                                0x00 = 传送带停止
                                0x01 = 传送带正转
                                0x02 = 传送带反转*/
  uint8_t conveyorBelts1:2;   /*2： 3bits: 1 号传送带状态，其值意义同传送带 0*/
  
  uint8_t parkingApron0:1;    /*4bits: 0 号停机坪状态：
                                0： 未检测到飞行器;
                                1： 检测到飞行器*/
  uint8_t parkingApron1:1;    /*5bits: 1 号停机坪状态*/
  uint8_t parkingApron2:1;    /*6bits: 2 号停机坪状态*/
  uint8_t parkingApron3:1;    /*7bits: 3 号停机坪状态*/
  
  tGpsData gpsData;           /*GPS 状态， 见 tGpsData 结构体定义*/
}tGameInfo;


/** 
  * @brief  实时血量变化信息(0x0002)
  */
typedef __packed struct
{
  uint8_t weakId:4;    /*0-3bits: 若变化类型为装甲伤害时：标识装甲 ID
                        0x00: 0 号装甲面 （前）
                        0x01： 1 号装甲面 （左）
                        0x02： 2 号装甲面 （后）
                        0x03： 3 号装甲面 （ 右）
                        0x04: 4 号装甲面 （上 1）
                        0x05: 5 号装甲面（ 上 2）*/
  
  uint8_t way:4;       /*4-7bits: 血量变化类型
                        0x0: 装甲伤害（受到攻击）
                        0x1：子弹超速扣血
                        0x2: 子弹超频扣血
                        0x3: 功率超限
                        0x4: 模块离线扣血
                        0x6: 普通犯规扣血
                        0xa: 获取加血神符*/
  
  uint16_t value;     /*血量变化值*/
}tRealBloodChangedData;


/** 
  * @brief  实时射击信息(0x0003)
  */
typedef __packed struct
{
  float realBulletShootSpeed; //子弹实时射速（ m/s）
  float realBulletShootFreq;  //子弹实时射频（ 发/s）
  float realGolfShootSpeed;   //高尔夫实时射速(m/s 英雄机器人)
  float realGolfShootFreq;    //高尔夫实时射频(发/s 英雄机器人)
}tRealShootData;


/** 
  * @brief  裁判系统信息汇总
  */
typedef struct
{
  tFrameHeader          frameHeader;
  uint16_t              rxCmdId;
  tGameInfo             gameInfo;
  tRealBloodChangedData realBloodChangedData;
  tRealShootData        realShootData;
  SelfCheckTypeDef      tSelfCheck;
}JudgementDataTypedef;


void judgementDataHandler(void);

#endif
