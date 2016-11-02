#ifndef __MPU6050_HMC5883L_DRIVER_H__
#define __MPU6050_HMC5883L_DRIVER_H__

typedef struct __MPU6050_RAW_Data__
{
    short Accel_X;  //寄存器原生X轴加速度表示值
    short Accel_Y;  //寄存器原生Y轴加速度表示值
    short Accel_Z;  //寄存器原生Z轴加速度表示值
    short Temp;     //寄存器原生温度表示值
    short Gyro_X;   //寄存器原生X轴陀螺仪表示值
    short Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    short Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
}MPU6050_REAL_DATA;

typedef struct __HMC5883L_RAW_Data__
{
    short Mag_X;  //寄存器原生X轴磁力计表示值
    short Mag_Y;  //寄存器原生Y轴磁力计表示值
    short Mag_Z;  //寄存器原生Z轴磁力计表示值

}HMC5883L_RAW_DATA;

typedef struct __HMC5883L_REAL_Data__
{
    float Mag_X;  //转换成实际的X轴加速度，
    float Mag_Y;  //转换成实际的Y轴加速度，
    float Mag_Z;  //转换成实际的Z轴加速度，
    float Yaw_Angle;   //转换成实际的X轴角加速度，
}HMC5883L_REAL_DATA;

extern MPU6050_RAW_DATA    MPU6050_Raw_Data; 
extern MPU6050_REAL_DATA   MPU6050_Real_Data;
extern HMC5883L_RAW_DATA   HMC5883L_Raw_Data;
extern HMC5883L_REAL_DATA   HMC5883L_Real_Data;

//定义MPU6050内部地址
#define	SMPLRT_DIV			0x19	//陀螺仪采样率 典型值 0X07 125Hz
#define	CONFIG					0x1A	//低通滤波频率 典型值 0x00 
#define	GYRO_CONFIG			0x1B	//陀螺仪自检及测量范围                 典型值 0x18 不自检 2000deg/s
#define	ACCEL_CONFIG		0x1C	//加速度计自检及测量范围及高通滤波频率 典型值 0x01 不自检 2G 5Hz

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A    //只读

#define	ACCEL_XOUT_H		0x3B
#define	ACCEL_XOUT_L		0x3C

#define	ACCEL_YOUT_H		0x3D
#define	ACCEL_YOUT_L		0x3E

#define	ACCEL_ZOUT_H		0x3F
#define	ACCEL_ZOUT_L		0x40
	
#define	TEMP_OUT_H			0x41
#define	TEMP_OUT_L			0x42

#define	GYRO_XOUT_H			0x43
#define	GYRO_XOUT_L			0x44	

#define	GYRO_YOUT_H			0x45
#define	GYRO_YOUT_L			0x46

#define	GYRO_ZOUT_H			0x47
#define	GYRO_ZOUT_L			0x48

#define	PWR_MGMT_1			0x6B	//电源管理 典型值 0x00 正常启用
#define	WHO_AM_I				0x75	//只读  默认读出应该是 MPU6050_ID = 0x68


//定义MPU6050信息
#define MPU6050_ID              0x68
#define MPU6050_DEVICE_ADDRESS  0xD0
#define MPU6050_DATA_START      ACCEL_XOUT_H   //由于数据存放地址是连续的，所以一并读出

//定义HMC5883L内部地址
#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAY_H         0x05
#define HMC5883L_RA_DATAY_L         0x06
#define HMC5883L_RA_DATAZ_H         0x07
#define HMC5883L_RA_DATAZ_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

//定义HMC5883L信息
#define HMC5883L_DEVICE_ADDRESS 0x3c
#define HMC5883L_ID 					  0x00
#define HMC5883L_DATA_START     HMC5883L_RA_DATAX_H

//MPU6050初始化及读取数据、校正
int MPU6050_Initialization(void);
int MPU6050_ReadData(void);
void MPU6050_Gyro_calibration(void);

//HMC5883L初始化及读取数据
int HMC5883L_Initialization(void);
int HMC5883L_ReadData(void);

#endif
