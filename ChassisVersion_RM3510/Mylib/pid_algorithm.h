#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__
#include <stm32f4xx.h>

#define ESC_MAX 1000.0

float Velocity_Control_820R(float current_velocity_820R,float target_velocity_820R);
float Position_Control_820R(float current_position_820R,float target_position_820R);

#endif
