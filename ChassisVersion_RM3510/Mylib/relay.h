#ifndef __RELAY_H__
#define __RELAY_H__

#define RELAY1_ON()    GPIO_SetBits(GPIOE,GPIO_Pin_12)
#define RELAY1_OFF()   GPIO_ResetBits(GPIOE,GPIO_Pin_12)

#define RELAY2_ON()    GPIO_SetBits(GPIOE,GPIO_Pin_13)
#define RELAY2_OFF()   GPIO_ResetBits(GPIOE,GPIO_Pin_13)

#define RELAY3_ON()    GPIO_SetBits(GPIOE,GPIO_Pin_14)
#define RELAY3_OFF()   GPIO_ResetBits(GPIOE,GPIO_Pin_14)

#define RELAY4_ON()    GPIO_SetBits(GPIOE,GPIO_Pin_15)
#define RELAY4_OFF()   GPIO_ResetBits(GPIOE,GPIO_Pin_15)

#define RELAY5_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_10)
#define RELAY5_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_10)

#define RELAY6_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_11)
#define RELAY6_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_11)

#define RELAY7_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define RELAY7_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_12)

#define RELAY8_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define RELAY8_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_13)

#define RELAY9_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_14)
#define RELAY9_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_14)

#define RELAY10_ON()   GPIO_SetBits(GPIOB,GPIO_Pin_15)
#define RELAY10_OFF()  GPIO_ResetBits(GPIOB,GPIO_Pin_15)

void Relay_Configuration(void);
void Relay_Test(void);

#endif 
