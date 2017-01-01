/************************************************************************************
  File Name     :  pid_algorithm.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  云台pitch（206）和yaw（205）的速度环和位置环的pid算法，
									 RM3510电机820R电调的的速度环和位置环的pid算法
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			  pid算法	
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  注释	
4			1.4       7/1				 yf     Velocity_Control_Shoot  拨弹电机速度环PID          
************************************************************************************/
#include "main.h"

#define GAP 0.0

/********************************************************************************
                           820R轴电调板的速度环控制
                      输入 820R轴当前速度 820R轴目标速度
*********************************************************************************/
float Velocity_Control_820R(float current_velocity_820R,float target_velocity_820R)
{
    const float l_p = ESC_820R_VEL_P;//7.0
    const float l_i = ESC_820R_VEL_I;//0.5
    const float l_d = ESC_820R_VEL_D;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_velocity_820R - current_velocity_820R;
    inte += error_l[1]; 
    
    output = error_l[1] * l_p 
            + inte * l_i 
            + (error_l[1] - error_l[0]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    		
    return output;
}
/********************************************************************************
                           820R电调板的位置环控制
                      输入 820R轴当前位置 820R轴目标位置
*********************************************************************************/
float Position_Control_820R(float current_position_820R,float target_position_820R)
{
    const float l_p = ESC_820R_POS_P;
		const float l_i = ESC_820R_POS_I;
		const float l_d = ESC_820R_POS_D;
    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_position_820R - current_position_820R;
    inte += error_l[1]; 
    
    output = error_l[1] * l_p 
            + inte * l_i 
            + (error_l[1] - error_l[0]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    		
    return output;
}


