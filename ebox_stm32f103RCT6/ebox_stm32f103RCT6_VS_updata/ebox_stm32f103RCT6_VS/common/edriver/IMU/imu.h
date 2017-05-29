#ifndef __IMU_H__
#define __IMU_H__
#include "sys.h"
#include <math.h>

//传感器原始数据
typedef struct  sensor_data
{
	short X;
	short Y;
	short Z;
}SENSOR_DATA;

//处理后的数据
typedef struct  imu_data
{
	float X;
	float Y;
	float Z;
}IMU_DATA;


void Get_MPU9250_DATA(void);//读取MPU9250数据
void AHRS_Dataprepare(void);//读取的ADC数据处理
void Acc_Correct(void);//加速度计校正
void Gyro_Correct(void);//陀螺仪校正
void Mag_Correct(void);//磁力计校正
void AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz);//姿态解算得出欧拉角
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);














#endif
