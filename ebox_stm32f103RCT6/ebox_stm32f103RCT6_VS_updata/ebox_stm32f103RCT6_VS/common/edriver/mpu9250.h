#/**
  ******************************************************************************
  * @file    mpu9250.h
  * @author  shentq
  * @version V1.2
  * @date    2017/06/11
  * @brief   
  ******************************************************************************
  * @attention
  *
  * No part of this software may be used for any commercial activities by any form 
  * or means, without the prior written consent of shentq. This specification is 
  * preliminary and is subject to change at any time without notice. shentq assumes
  * no responsibility for any errors contained herein.
  * <h2><center>&copy; Copyright 2015 shentq. All Rights Reserved.</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __MPU9250_H
#define __MPU9250_H

#include "ebox.h"
#include "math.h"
/*ģʽѡ��0Ϊmpu6500ģʽ��1ΪAK8963ģʽ*/
#define MPU6500   0
#define AK8963    1

/*MPU6050 Register Address ------------------------------------------------------------*/
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18

#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I			0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define SLAVE_ADDRESS	0xD0	//IICд��ʱ�ĵ�ַ�ֽ�����

//AK9863
#define INT_PIN_CFG         0x37
#define SLAVEAWRITE         0xD1
#define RA_MAG_ADDRESS		0x18
#define RA_MAG_INFO		    0x01
#define RA_MAG_ST1	        0x02
#define RA_MAG_XOUT_L		0x03
#define RA_MAG_XOUT_H		0x04
#define RA_MAG_YOUT_L		0x05
#define RA_MAG_YOUT_H		0x06
#define RA_MAG_ZOUT_L		0x07
#define RA_MAG_ZOUT_H		0x08
#define RA_MAG_ST2		    0x09
#define MAG_CNTL1           0x0A
#define MAG_CNTL2           0x0B
#define MAG_TEST1           0x0D
//AHRS�㷨����
#define betaDef		0.2f		// 2 * proportional gain
/*
#define sampleFreq	125.0f		// sample frequency in Hz  ������ 100 HZ  10ms  �޸Ĵ�Ƶ�ʿ����ӱ仯�ٶ�
#define Kp 2.25f	//��������֧��������accellrometer/magnetometer           //0.6 0.7       2.5
#define Ki 0.0055f //��������ִ�����������ǵ��ν�gyroscopeases           //0.002  0.004   0.005
#define halfT 0.004f//�������ڵ�һ�룬������Ϊ10ms,��һ��Ϊ0.005s        // 1      1      0.2
*/
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

template<typename T>
class Mpu9250
{
public:
	float sample;   //������
	Mpu9250(T *m_i2c)
	{
		i2c = m_i2c;
	}
	void        begin(uint32_t speed=1);
	//�趨ģʽ
	void        mode(u8 mode);
	int16_t 	getData2byte(uint8_t reg_address);
	//�ֽڶ�ȡ�����ڶ�ȡ�ض��Ĵ���
	int8_t   	getData1byte(uint8_t reg_address);
	//������ַ�Ĵ������ݶ�ȡ
	int8_t 		getData(uint8_t reg_address, int16_t *buf, uint8_t num_to_read);
	//���Ĵ���д����
	void        writeData(u8 reg_address, u8 data);
	//��ȡ�豸id
	void        getId(uint8_t *id);
	//�κ�ģʽ�¶�MPU6050�Ĵ����Ķ�ȡ��������ȡ0x37�����Լ���Ƿ���ȷ�л���������ģʽ
	uint8_t     getDataMpu(uint8_t reg_address);
	T *getI2c(void);



private:
	T *i2c;
	uint32_t   speed;
	u8 salve_flag;
	u8 salve_adress;
	

};

#endif

template<typename T>
class Mpu9250Ahrs :public Mpu9250<T>
{
public:
	//���캯��
	Mpu9250Ahrs(T *i2c);

   //	void begin(uint_t64 speed);    //����Mpu9250��begin����

	//����ˢ����
	void setTime(uint16_t mtime);

	//ԭʼ���ݻ�ȡ�����ɹ�����-1.-2���ɹ�û���趨����ֵ��δ֪����ֵ
	int getMpu9250Data(void);

	//���Ժ��������Գ�ʼ�����Ƿ�ɹ���ȡ
	void getDataBuf(int16_t *mpu, int16_t *AK);

	//ADCת������ADC����ת��Ϊֱ������
	void ahrsDataPrepare(void);

	//ADC����ת������
	void getDataAdc(float *mpu, float *AK);

	//���ٶȼ�У��
	void accCorrect(void);

	//������У��
	void gyroCorrect(void);

	//������У��
	void magCorrect(float x,float y);

	//��̬����ó�ŷ���ǣ����ⲿ�������
	void ahrsUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	
	//�����ڲ�����
	void ahrsUpdate(void);

	//��ȡŷ����
	void getDataAhrs(float *Pitch, float *Roll, float *Yaw);

	//���Դ��룺����q0,q1,q2,q3
	void getDataQ(float *q);

	//������ƽ����
	float invSqrt(float x);

	//�������ã����������
	void setParameter(float m_kp, float m_ki, float m_sample,uint16_t mtime);

	//������֤����֤����������ȷ���:����ϵ����΢��ϵ���������ʣ������ʶ�Ӧ�ļĴ������ò������������ڵ�һ��
	void getParameter(float *m_kp, float *m_ki, float *m_samplefreq, uint8_t *sampleH, float *m_halfT);

private:
	//����ǰ����
	SENSOR_DATA Gyrobuf;//������
	SENSOR_DATA Accbuf; //���ٶ�
	SENSOR_DATA Magbuf;//������

	SENSOR_DATA Accoffset;//���ٶ�ƫ����
	SENSOR_DATA Gyrooffset;//������ƫ����
	SENSOR_DATA Magoffset;//������ƫ����

						  //����������
	IMU_DATA GyroFinal;
	IMU_DATA AccFinal;
	IMU_DATA MagFinal;
	//���ղ���
	float temp;//�¶�
	float Pitch;
	float Roll;
	float Yaw;
	float Pitch_off;
	float Roll_off;
	float Yaw_off;


	float q0, q1, q2, q3;	// quaternion of sensor frame relativ
	float exInt, eyInt, ezInt;
	float Ki, Kp;
	float halfT;
	float  sampleFreq;
	T *i2c;
	uint16_t time;
	float mx;
	float my;
};



template<typename T>
void Mpu9250<T>::begin(uint32_t speed)
{
	this->speed = speed;
	i2c->take_i2c_right(this->speed);
	i2c->begin(this->speed);
	i2c->write_byte(SLAVE_ADDRESS, PWR_MGMT_1, 0x80);    //��λ
	i2c->write_byte(SLAVE_ADDRESS, PWR_MGMT_1, 0x00);   //����
	i2c->write_byte(SLAVE_ADDRESS, PWR_MGMT_1, 0x01);
	i2c->write_byte(SLAVE_ADDRESS, SMPLRT_DIV, sample); //166hz����
	i2c->write_byte(SLAVE_ADDRESS, CONFIG, 0x07);    //10hz�˲�
	i2c->write_byte(SLAVE_ADDRESS, GYRO_CONFIG, 0x18);
	i2c->write_byte(SLAVE_ADDRESS, ACCEL_CONFIG, 0x00);//2g
	i2c->write_byte(SLAVE_ADDRESS, PWR_MGMT_1, 0x01);
	i2c->write_byte(SLAVE_ADDRESS, SMPLRT_DIV, 0x09);
	i2c->write_byte(SLAVE_ADDRESS, CONFIG, 0x06);
	i2c->write_byte(SLAVE_ADDRESS, GYRO_CONFIG, 0x18);
	i2c->write_byte(SLAVE_ADDRESS, ACCEL_CONFIG, 0x01);
	i2c->release_i2c_right();
		
}
template<typename T>
T *Mpu9250<T>::getI2c(void)
{
	return this->i2c;
}

template<typename T>
void Mpu9250<T>::getId(uint8_t *id)
{
	u8 adress;
	if (salve_adress == SLAVE_ADDRESS)
		adress = WHO_AM_I;
	if (salve_adress == RA_MAG_ADDRESS)
		adress = 0x00;
	i2c->take_i2c_right(speed);
	i2c->read_byte(salve_adress, adress, id, 1);
	i2c->release_i2c_right();
};
template<typename T>
int16_t  Mpu9250<T>::getData2byte(uint8_t reg_address)
{
	uint8_t tmp[2];
	i2c->take_i2c_right(speed);
	i2c->read_byte(salve_adress, reg_address, tmp, 2);
	i2c->release_i2c_right();
	if (salve_adress == SLAVE_ADDRESS)
		return ((tmp[0] << 8) + tmp[1]);
	if (salve_adress == RA_MAG_ADDRESS)
		return ((tmp[1] << 8) + tmp[0]);
}
template<typename T>
int8_t  Mpu9250<T>::getData1byte(uint8_t reg_address)
{
	uint8_t tmp[1];
	i2c->take_i2c_right(speed);
	i2c->read_byte(salve_adress, reg_address, tmp, 1);
	i2c->release_i2c_right();
	return (tmp[0]);

}
template<typename T>
int8_t  Mpu9250<T>::getData(uint8_t reg_address, int16_t *buf, uint8_t num_to_read)
{
	uint8_t i, readnum;
	uint8_t tmpbuf[20];

	i2c->take_i2c_right(speed);
	readnum = i2c->read_byte(salve_adress, reg_address, tmpbuf, num_to_read * 2);
	i2c->release_i2c_right();

	for (i = 0; i < readnum / 2; i++)
	{
		if (salve_adress == SLAVE_ADDRESS)
			*buf++ = (tmpbuf[i * 2 + 0] << 8) + (tmpbuf[i * 2 + 1]);
		if (salve_adress == RA_MAG_ADDRESS)
			*buf++ = (tmpbuf[i * 2 + 1] << 8) + (tmpbuf[i * 2 + 0]);
	}
	return readnum / 2;
}

template<typename T>
void  Mpu9250<T>::writeData(u8 reg_address, u8 data)
{
	i2c->take_i2c_right(speed);
	i2c->write_byte(salve_adress, reg_address, data);
	i2c->release_i2c_right();
}
template<typename T>
u8 Mpu9250<T>::getDataMpu(uint8_t reg_address)
{
	uint8_t tmp[1];
	i2c->take_i2c_right(speed);
	i2c->read_byte(SLAVE_ADDRESS, reg_address, tmp, 1);
	i2c->release_i2c_right();
	return (tmp[0]);
}
template<typename T>
void Mpu9250<T>::mode(u8 mode)
{
	if (mode)
	{
		salve_flag = 1;
		i2c->take_i2c_right(this->speed);
		i2c->write_byte(SLAVE_ADDRESS, INT_PIN_CFG, 0x02);
		i2c->release_i2c_right();
		salve_adress = RA_MAG_ADDRESS;
	}

	if (!mode)
	{
		salve_flag = 0;
		i2c->take_i2c_right(this->speed);
		i2c->write_byte(SLAVE_ADDRESS, INT_PIN_CFG, 0x00);
		i2c->release_i2c_right();
		salve_adress = SLAVE_ADDRESS;
	}


}



//��������������
template<typename T>
Mpu9250Ahrs<T>::Mpu9250Ahrs(T *i2c):Mpu9250<T>(i2c)
{

q0 = 1.0f;
q1 = 0.0f;
q2 = 0.0f;
q3 = 0.0f;
Pitch_off = 0.00f;
Roll_off = 0.00f;
Yaw_off = 0.00f;
exInt = 0;
eyInt = 0;
ezInt = 0;
time = 1000;
mx = 24.905;
my = 15.647;

}


template<typename T>
void Mpu9250Ahrs<T>::setTime(uint16_t mtime)
{
	time = mtime;
}
template<typename T>
int Mpu9250Ahrs<T>::getMpu9250Data(void)
{
u8 id;
u8 AK_id;
int16_t temp[7];
int16_t AK_temp[3];
//���ݶ���
this->mode(MPU6500);
this->getId(&id);
if (id == 0x73)
	{
		this->Accbuf.X = this->getData2byte(ACCEL_XOUT_H);
		this->Accbuf.Y = this->getData2byte(ACCEL_YOUT_H);
		this->Accbuf.Z = this->getData2byte(ACCEL_ZOUT_H);
		this->Gyrobuf.X = this->getData2byte(GYRO_XOUT_H);
		this->Gyrobuf.Y = this->getData2byte(GYRO_YOUT_H);
		this->Gyrobuf.Z = this->getData2byte(GYRO_ZOUT_H);
	}

else
    return -1;
//delay_us(100);
this->mode(AK8963);
  delay_us(time);
this->getId(&AK_id);
if (AK_id == 0x48)
this->writeData(MAG_CNTL1, 0x11);
else
return -2;
this->writeData(MAG_TEST1, 0x08);
   this->Magbuf.X = this->getData2byte(RA_MAG_XOUT_L) + 65;
   this->Magbuf.Y = this->getData2byte(RA_MAG_YOUT_L) - 65;
   this->Magbuf.Z = this->getData2byte(RA_MAG_ZOUT_L);
this->getData2byte(RA_MAG_ST2);

/*
//���ݴ���
//���ٶ�
this->Accbuf.X = temp[0];
this->Accbuf.Y = temp[1];
this->Accbuf.Z = temp[2];
//������
this->Gyrobuf.X = temp[4];
this->Gyrobuf.Y = temp[5];
this->Gyrobuf.Z = temp[6];
//������
this->Magbuf.X = AK_temp[0];
this->Magbuf.Y = AK_temp[1];
this->Magbuf.Z = AK_temp[2];

this->Magbuf.X = AK_temp[0] + 65;
this->Magbuf.Y = AK_temp[1] - 65;
//this->Magbuf.Y = AK_temp[1]-70;
this->Magbuf.Z = AK_temp[2];
*/
return 0;
}

template<typename T>
void Mpu9250Ahrs<T>::getDataBuf(int16_t *mpu, int16_t *AK)
{
*mpu++ = this->Accbuf.X;
*mpu++ = this->Accbuf.Y;
*mpu++ = this->Accbuf.Z;

*mpu++ = this->Gyrobuf.X;
*mpu++ = this->Gyrobuf.Y;
*mpu = this->Gyrobuf.Z;

*AK++ = this->Magbuf.X;
*AK++ = this->Magbuf.Y;
*AK = this->Magbuf.Z;

}

template<typename T>
void Mpu9250Ahrs<T>::ahrsDataPrepare()
{
this->getMpu9250Data();//�Ȼ�ȡ����
//16.4 = 2^16/4000 lsb ��/s     1/16.4=0.061     0.0174 = 3.14/180
//���������ݴ�ADCת��Ϊ����ÿ��(������Ҫ��ȥƫ��ֵ)
GyroFinal.X = (Gyrobuf.X - Gyrooffset.X)*0.061*0.0174;
GyroFinal.Y = (Gyrobuf.Y - Gyrooffset.Y)*0.061*0.0174;
GyroFinal.Z = (Gyrobuf.Z - Gyrooffset.Z)*0.061*0.0174;		//����ֵ��ȥ��׼ֵ���Ե�λ�����������ǽ��ٶ�

//+-2g,2^16/4=16384lsb/g--0.061mg/lsb
//�˴�0.0098�ǣ�(9.8m/s^2)/1000,����mg��m/s^2
AccFinal.X = (float)((Accbuf.X - Accoffset.X)*0.061)*0.0098;
AccFinal.Y = (float)((Accbuf.Y - Accoffset.Y)*0.061)*0.0098;
AccFinal.Z = (float)((Accbuf.Z - Accoffset.Z)*0.061)*0.0098;

//��4800uT 2^16/9600 = 6.83lsb/uT     1/6.83 = 0.1465
//�ش�ǿ��Ϊ 5-6 x 10^(-5) T = 50 - 60 uT
MagFinal.X = (float)(Magbuf.X - Magoffset.X)*0.1465 - mx;
MagFinal.Y = (float)(Magbuf.Y - Magoffset.Y)*0.1465 - my;
MagFinal.Z = (float)(Magbuf.Z - Magoffset.Z)*0.1465;
/*
MagFinal.X = MagFinal.X 
MagFinal.Y = MagFinal.Y 
MagFinal.Z = MagFinal.Z;
*/
}

template<typename T>
void Mpu9250Ahrs<T>::accCorrect()
{
u8 i = 0;
u8 numAcc = 200;//ȡ100���ۼ���

int Angleaccx = 0;  //���ٶȼ�У���м����
int Angleaccy = 0;
int Angleaccz = 0;

for (i = 0; i<numAcc; i++)
{
getMpu9250Data();
Angleaccx += Accbuf.X;
Angleaccy += Accbuf.Y;
Angleaccz += Accbuf.Z;
delay_ms(2);
}
Accoffset.X = Angleaccx / numAcc;
Accoffset.Y = Angleaccy / numAcc;
Accoffset.Z = Angleaccy / numAcc;				   //�õ����ٶȼƻ�׼
}

template<typename T>
void Mpu9250Ahrs<T>::gyroCorrect()
{
unsigned char i = 0;
unsigned char numGyro = 200;

int Gyrox = 0;
int Gyroy = 0;
int Gyroz = 0;							  //������У���м����

for (i = 0; i<numGyro; i++)
{
getMpu9250Data();
Gyrox += Gyrobuf.X;
Gyroy += Gyrobuf.Y;
Gyroz += Gyrobuf.Z;
delay_ms(2);
}

Gyrooffset.X = Gyrox / numGyro;
Gyrooffset.Y = Gyroy / numGyro;
Gyrooffset.Z = Gyroz / numGyro;
}

template<typename T>
void Mpu9250Ahrs<T>::magCorrect(float x, float y)
{
	mx = x;
	my = y;
}

template<typename T>
void Mpu9250Ahrs<T>::getDataAdc(float *mpu, float *AK)
{
*mpu++ = this->AccFinal.X;
*mpu++ = this->AccFinal.Y;
*mpu++ = this->AccFinal.Z;
*mpu++ = this->GyroFinal.X;
*mpu++ = this->GyroFinal.Y;
*mpu = this->GyroFinal.Z;

*AK++ = this->MagFinal.X;
*AK++ = this->MagFinal.Y;
*AK = this->MagFinal.Z;


}

template<typename T>
float Mpu9250Ahrs<T>::invSqrt(float x)
{
float halfx = 0.5f * x;
float y = x;
long i = *(long*)&y;
i = 0x5f3759df - (i >> 1);
y = *(float*)&i;
y = y * (1.5f - (halfx * y * y));
return y;
}

template<typename T>
void Mpu9250Ahrs<T>::ahrsUpdate(void)
{
float ax = this->AccFinal.X;
float ay = this->AccFinal.Y;
float az = this->AccFinal.Z;

float gx = this->GyroFinal.X;
float gy = this->GyroFinal.Y;
float gz = this->GyroFinal.Z;

float mx = this->MagFinal.X;
float my = this->MagFinal.Y;
float mz = this->MagFinal.Z;


float recipNorm;
float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
float hx, hy, hz, bx, bz;
float vx, vy, vz, wx, wy, wz;
float ex, ey, ez;
float qa, qb, qc;
float integralFBx, integralFBy, integralFBz;
if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
{

// Normalise accelerometer measurement
//�������ļ��ٶȲ���ֵ
recipNorm = invSqrt(ax * ax + ay * ay + az * az);
ax *= recipNorm;
ay *= recipNorm;
az *= recipNorm;

// Normalise magnetometer measurement
//�������Ĵ����Ʋ���ֵ
recipNorm = invSqrt(mx * mx + my * my + mz * mz);
mx *= recipNorm;
my *= recipNorm;
mz *= recipNorm;

//Ԥ�Ƚ�����Ԫ���������㣬�Ա����ظ����������Ч�����⡣
// Auxiliary variables to avoid repeated arithmetic
q0q0 = q0 * q0;
q0q1 = q0 * q1;
q0q2 = q0 * q2;
q0q3 = q0 * q3;
q1q1 = q1 * q1;
q1q2 = q1 * q2;
q1q3 = q1 * q3;
q2q2 = q2 * q2;
q2q3 = q2 * q3;
q3q3 = q3 * q3;

hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
bx = sqrt(hx * hx + hy * hy);
bz = hz;

vx = q1q3 - q0q2;
vy = q0q1 + q2q3;
vz = q0q0 - 0.5f + q3q3;
wx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
wy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
wz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

//ʹ�ò�������������͵ش���
// Error is sum of cross product between estimated direction and measured direction of field vectors
ex = (ay * vz - az * vy) + (my * wz - mz * wy) / 1;
ey = (az * vx - ax * vz) + (mz * wx - mx * wz) / 1;
ez = (ax * vy - ay * vx) + (mx * wy - my * wx) / 1;

//�������л���
exInt += Ki * ex * (1.0f / sampleFreq); // integral error scaled by Ki
eyInt += Ki * ey * (1.0f / sampleFreq);
ezInt += Ki * ez * (1.0f / sampleFreq);

//����ʵ�ļ��ٶȲ���ֵ��һ�����������������ǣ�0������ȫ���������ǣ�1������ȫ���μ��ٶȣ�����1��
gx = gx + Kp*ex + exInt;
gy = gy + Kp*ey + eyInt;
gz = gz + Kp*ez + ezInt;

/*
qa = q0;
qb = q1;
qc = q2;
q0 += (-qb * gx - qc * gy - q3 * gz);
q1 += (qa * gx + qc * gz - q3 * gy);
q2 += (qa * gy - qb * gz + q3 * gx);
q3 += (qa * gz + qb * gy - qc * gx);
*/

qa = q0;
qb = q1;
qc = q2;
q0 += (-qb * gx - qc * gy - q3 * gz)*halfT;
q1 += (qa * gx + qc * gz - q3 * gy)*halfT;
q2 += (qa * gy - qb * gz + q3 * gx)*halfT;
q3 += (qa * gz + qb * gy - qc * gx)*halfT;

// Normalise quaternion

recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
q0 *= recipNorm;
q1 *= recipNorm;
q2 *= recipNorm;
q3 *= recipNorm;


//��Ԫ��ת����ŷ����
/*
Pitch = asin(2 * q0*q2 - 2 * q1*q3) / 3.14 * 180;
Roll = atan2(2 * q0*q1 + 2 * q2*q3, 1 - 2 * q1*q1 - 2 * q2*q2) / 3.14 * 180;
Yaw = atan2(2 * q0*q3 + 2 * q1*q2, 1 - 2 * q2*q2 - 2 * q3*q3) / 3.14 * 180;
*/

Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2); //�����ǣ���y��ת��
Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); //�����ǣ���x��ת��
//0.9��0.1������ϵ��������5.73=0.1*57.3������57.3��Ϊ�˽�����ת��Ϊ�Ƕ�
Yaw = -(0.85 * (-Yaw + gz * 2 * halfT) + 0.15*57.3 * atan2(mx*cos(Roll) + my*sin(Roll)*sin(Pitch) + mz*sin(Roll)*cos(Pitch), my*cos(Pitch) - mz*sin(Pitch)));
Pitch = Pitch * 57.3;
Roll = Roll * 57.3;


   }

}
template<typename T>
void Mpu9250Ahrs<T>::ahrsUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{

float recipNorm;
float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
float hx, hy, hz, bx, bz;
float vx, vy, vz, wx, wy, wz;
float ex, ey, ez;
float qa, qb, qc;
float integralFBx, integralFBy, integralFBz;
if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
{

// Normalise accelerometer measurement
//�������ļ��ٶȲ���ֵ
recipNorm = invSqrt(ax * ax + ay * ay + az * az);
ax *= recipNorm;
ay *= recipNorm;
az *= recipNorm;

// Normalise magnetometer measurement
//�������Ĵ����Ʋ���ֵ
recipNorm = invSqrt(mx * mx + my * my + mz * mz);
mx *= recipNorm;
my *= recipNorm;
mz *= recipNorm;

//Ԥ�Ƚ�����Ԫ���������㣬�Ա����ظ����������Ч�����⡣
// Auxiliary variables to avoid repeated arithmetic
q0q0 = q0 * q0;
q0q1 = q0 * q1;
q0q2 = q0 * q2;
q0q3 = q0 * q3;
q1q1 = q1 * q1;
q1q2 = q1 * q2;
q1q3 = q1 * q3;
q2q2 = q2 * q2;
q2q3 = q2 * q3;
q3q3 = q3 * q3;

hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
bx = sqrt(hx * hx + hy * hy);
bz = hz;

vx = q1q3 - q0q2;
vy = q0q1 + q2q3;
vz = q0q0 - 0.5f + q3q3;
wx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
wy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
wz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

//ʹ�ò�������������͵ش���
// Error is sum of cross product between estimated direction and measured direction of field vectors
ex = (ay * vz - az * vy) + (my * wz - mz * wy) / 1;
ey = (az * vx - ax * vz) + (mz * wx - mx * wz) / 1;
ez = (ax * vy - ay * vx) + (mx * wy - my * wx) / 1;

//�������л���
exInt += Ki * ex * (1.0f / sampleFreq); // integral error scaled by Ki
eyInt += Ki * ey * (1.0f / sampleFreq);
ezInt += Ki * ez * (1.0f / sampleFreq);

//����ʵ�ļ��ٶȲ���ֵ��һ�����������������ǣ�0������ȫ���������ǣ�1������ȫ���μ��ٶȣ�����1��
gx = gx + Kp*ex + exInt;
gy = gy + Kp*ey + eyInt;
gz = gz + Kp*ez + ezInt;

/*
qa = q0;
qb = q1;
qc = q2;
q0 += (-qb * gx - qc * gy - q3 * gz);
q1 += (qa * gx + qc * gz - q3 * gy);
q2 += (qa * gy - qb * gz + q3 * gx);
q3 += (qa * gz + qb * gy - qc * gx);
*/
///*

qa = q0;
qb = q1;
qc = q2;
q0 += (-qb * gx - qc * gy - q3 * gz)*halfT;
q1 += (qa * gx + qc * gz - q3 * gy)*halfT;
q2 += (qa * gy - qb * gz + q3 * gx)*halfT;
q3 += (qa * gz + qb * gy - qc * gx)*halfT;

// Normalise quaternion

recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
q0 *= recipNorm;
q1 *= recipNorm;
q2 *= recipNorm;
q3 *= recipNorm;


//��Ԫ��ת����ŷ����
/*
Pitch = asin(2 * q0*q2 - 2 * q1*q3) / 3.14 * 180;
Roll = atan2(2 * q0*q1 + 2 * q2*q3, 1 - 2 * q1*q1 - 2 * q2*q2) / 3.14 * 180;
Yaw = atan2(2 * q0*q3 + 2 * q1*q2, 1 - 2 * q2*q2 - 2 * q3*q3) / 3.14 * 180;
*/


Pitch = asin(-2 * q1 * q3 + 2 * q0 * q2); //�����ǣ���y��ת��
Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); //�����ǣ���x��ת��
//0.9��0.1������ϵ��������5.73=0.1*57.3������57.3��Ϊ�˽�����ת��Ϊ�Ƕ�
Yaw = -(0.85 * (-Yaw + gz * 2 * halfT) + 0.15*57.3 * atan2(mx*cos(Roll) + my*sin(Roll)*sin(Pitch) + mz*sin(Roll)*cos(Pitch), my*cos(Pitch) - mz*sin(Pitch)));
Pitch = Pitch * 57.3;
Roll = Roll * 57.3;


   }

}

template<typename T>
void Mpu9250Ahrs<T>::setParameter(float m_kp, float m_ki, float m_sample,uint16_t mtime)
{
Kp = m_kp;;
Ki = m_ki;
sampleFreq = m_sample;
Mpu9250<T>::sample = int(1000 / m_sample - 1);
halfT = 1 / (2 * m_sample);
time = mtime;
}

template<typename T>
void Mpu9250Ahrs<T>::getParameter(float *m_kp, float *m_ki, float *m_samplefreq, uint8_t *sampleH, float *m_halfT)
{
*m_kp = this->Kp;
*m_ki = this->Ki;
*m_samplefreq = this->sampleFreq;
*sampleH = this->sample;
*m_halfT = halfT;

}

template<typename T>
void Mpu9250Ahrs<T>::getDataAhrs(float *m_Pitch, float *m_Roll, float *m_Yaw)
{
this->ahrsDataPrepare();
this->ahrsUpdate();
*m_Pitch = Pitch - Pitch_off;
*m_Roll = Roll - Roll_off;
*m_Yaw = Yaw - Yaw_off;
}

template<typename T>
void Mpu9250Ahrs<T>::getDataQ(float *q)
{
*q++ = q0;
*q++ = q1;
*q++ = q2;
*q = q3;
}

/*
template<typename T>
void Mpu9250Ahrs<T>::updateData(void)
{
//this->Pitch_off = Pitch;
//this->Roll_off = Roll;
this->Yaw_off = Yaw;
}
*/
