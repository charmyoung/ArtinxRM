#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__
#include <stm32f4xx.h>

#define ESC_MAX 3000.0

float Velocity_Control_206(float current_velocity_206,float target_velocity_206);
float Position_Control_206(float current_position_206,float target_position_206);


float Velocity_Control_205(float current_velocity_205,float target_velocity_205);
float Position_Control_205(float current_position_205,float target_position_205);

float Velocity_Control_820R(float current_velocity_820R,float target_velocity_820R);
float Position_Control_820R(float current_position_820R,float target_position_820R);

float Velocity_Control_Shoot(float current_velocity_Shoot,float target_velocity_Shoot);
#endif
