#ifndef _SDBUS_H_
#define _SDBUS_H_
#include <stdint.h>

typedef struct{
double xf;
double xtr;
double xrr;
	
double w1;
double w2;
double w3;
double w4;
}SDBUS;

extern SDBUS sdbus;

void SDBUS_Enc(const SDBUS* sdbus,unsigned char* sdbuf);
void SDBUS_Dec(SDBUS* sdbus,const unsigned char* sdbuf);
#endif
