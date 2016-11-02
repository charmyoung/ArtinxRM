#ifndef  __MPU6050_HMC5883L_PROCESS_H__
#define __MPU6050_HMC5883L_PROCESS_H__

typedef struct __ACCEL_AVERAGE_DATA__
{
    float X;
    float Y;
    float Z;
}ACCEL_AVERAGE_DATA;

typedef struct __GYRO_RADIAN_DATA__
{
    float X;
    float Y;
    float Z;
}GYRO_RADIAN_DATA;

typedef struct __AHRS_DATA__
{	
		float Acce_Nav_x;
	  float Acce_Nav_y;
		float Acce_Nav_z;
	
		float Vel_Nav_x;
	  float Vel_Nav_y;
		float Vel_Nav_z;
	
		float Pos_Nav_x;
		float Pos_Nav_y;
		float Pos_Nav_z;
	
		float q0;
		float q1;
		float q2;
		float q3;
	
		float EulerAngle_Pitch;
    float EulerAngle_Roll;
    float EulerAngle_Yaw;
}AHRS_DATA;


extern ACCEL_AVERAGE_DATA   Accel_Raw_Average_Data; 
extern GYRO_RADIAN_DATA     Gyro_Radian_Data;
extern AHRS_DATA        AHRS_Data;

void MPU6050_Data_Filter(void);
float invSqrt(float x);
void q_initialization(void);
void AHRS_Calculate( 					float gyro_x,
                              float gyro_y,
                              float gyro_z,
                              float accel_x,
                              float accel_y,
                              float accel_z,
															float mag_x,
															float mag_y,
															float mag_z);
#endif
