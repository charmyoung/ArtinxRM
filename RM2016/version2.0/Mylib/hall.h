#ifndef __HALL_H__
#define __HALL_H__

void Hall_Configuration(void);

extern int direction;          //正转为1，反转为-1
extern unsigned int BLDC_PWM;  //电机转动占空比，满值为1000，但到不了100%

#endif
