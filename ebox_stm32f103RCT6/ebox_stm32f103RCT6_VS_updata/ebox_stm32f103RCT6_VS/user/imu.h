#ifndef __IMU_H__
#define __IMU_H__
#include "sys.h"
#include <math.h>

//������ԭʼ����
typedef struct  sensor_data
{
	short X;
	short Y;
	short Z;
}SENSOR_DATA;

//����������
typedef struct  imu_data
{
	float X;
	float Y;
	float Z;
}IMU_DATA;


void Get_MPU9250_DATA(void);//��ȡMPU9250����
void AHRS_Dataprepare(void);//��ȡ��ADC���ݴ���
void Acc_Correct(void);//���ٶȼ�У��
void Gyro_Correct(void);//������У��
void Mag_Correct(void);//������У��
void AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz);//��̬����ó�ŷ����
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);














#endif
