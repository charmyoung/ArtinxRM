#ifndef _SDBUS_H_
#define _SDBUS_H_
#include <stdint.h>

typedef struct{
int PitchAngle;
int YawAngle;	
}SDBUS;

extern SDBUS sdbus;

void SDBUS_Enc(const SDBUS* sdbus,unsigned char* sdbuf);
void SDBUS_Dec(SDBUS* sdbus,const unsigned char* sdbuf);
#endif
