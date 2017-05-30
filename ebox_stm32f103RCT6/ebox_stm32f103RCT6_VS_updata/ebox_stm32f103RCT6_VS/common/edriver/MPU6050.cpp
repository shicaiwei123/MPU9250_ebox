/**
  ******************************************************************************
  * @file    mpu6050.cpp
  * @author  shentq
  * @version V1.2
  * @date    2016/08/14
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


/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#

void Mpu9250::begin(uint32_t speed)
{
    this->speed = speed;
    i2c->take_i2c_right(this->speed);
    i2c->begin(this->speed);
	i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0x80);    //复位
	i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0x00);   //唤醒
    i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0x01);
    i2c->write_byte(SLAVEADDRESS, SMPLRT_DIV, 0x09); //100hz采样
    i2c->write_byte(SLAVEADDRESS, CONFIG, 0x03);    //41hz滤波
    i2c->write_byte(SLAVEADDRESS, GYRO_CONFIG, 0x18);
    i2c->write_byte(SLAVEADDRESS, ACCEL_CONFIG, 0x00);//2g
    i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0x01); 
    i2c->write_byte(SLAVEADDRESS, SMPLRT_DIV, 0x09);
    i2c->write_byte(SLAVEADDRESS, CONFIG, 0x06);
    i2c->write_byte(SLAVEADDRESS, GYRO_CONFIG, 0x18);
    i2c->write_byte(SLAVEADDRESS, ACCEL_CONFIG, 0x01);
    i2c->release_i2c_right();
}
void Mpu9250::get_id(uint8_t *id)
{
	u8 adress;
	if (salve_adress == SLAVEADDRESS)
		adress = WHO_AM_I;
	if (salve_adress == RA_MAG_ADDRESS)
		adress = 0x00;
    i2c->take_i2c_right(speed);
    i2c->read_byte(salve_adress, adress, id, 1);
    i2c->release_i2c_right();
};

int16_t  Mpu9250::get_data_2byte(uint8_t reg_address)
{
    uint8_t tmp[2];
    i2c->take_i2c_right(speed);
    i2c->read_byte(salve_adress, reg_address, tmp, 2);
    i2c->release_i2c_right();
    if(salve_adress== SLAVEADDRESS)
        return ((tmp[0] << 8) + tmp[1]);
	if (salve_adress == RA_MAG_ADDRESS)
		return ((tmp[1] << 8) + tmp[0]);
}
int8_t  Mpu9250::get_data_1byte(uint8_t reg_address)
{
	uint8_t tmp[1];
	i2c->take_i2c_right(speed);
	i2c->read_byte(salve_adress, reg_address, tmp, 1);
	i2c->release_i2c_right();
	return (tmp[0]);

}
int8_t  Mpu9250::get_data(uint8_t reg_address, int16_t *buf, uint8_t num_to_read)
{
    uint8_t i, readnum;
    uint8_t tmpbuf[20];

    i2c->take_i2c_right(speed);
    readnum = i2c->read_byte(salve_adress, reg_address, tmpbuf, num_to_read * 2);
    i2c->release_i2c_right();

    for(i = 0; i < readnum / 2; i++)
    {
		if (salve_adress == SLAVEADDRESS)
            *buf++ = (tmpbuf[i * 2 + 0] << 8) + (tmpbuf[i * 2 + 1]);
		if (salve_adress == RA_MAG_ADDRESS)
            *buf++ = (tmpbuf[i * 2 + 1] << 8) + (tmpbuf[i * 2 + 0]);
	}
    return readnum / 2;
}


void  Mpu9250::write_data(u8 reg_address, u8 data)
{
	i2c->take_i2c_right(speed);
	i2c->write_byte(salve_adress, reg_address, data);
	i2c->release_i2c_right();
}

u8 Mpu9250::get_data_MPU(uint8_t reg_address)
{
	uint8_t tmp[1];
	i2c->take_i2c_right(speed);
	i2c->read_byte(SLAVEADDRESS, reg_address, tmp, 1);
	i2c->release_i2c_right();
	return (tmp[0]);
}
void Mpu9250::mode(u8 mode)
{
	if (mode)
	{
		salve_flag = 1;
		i2c->take_i2c_right(this->speed);
		i2c->write_byte(SLAVEADDRESS, INT_PIN_CFG, 0x02);
		i2c->release_i2c_right();
		salve_adress = RA_MAG_ADDRESS;
	}

	if (!mode)
	{
		salve_flag = 0;
		i2c->take_i2c_right(this->speed);
		i2c->write_byte(SLAVEADDRESS, INT_PIN_CFG, 0x00);
		i2c->release_i2c_right();
		salve_adress = SLAVEADDRESS;
	}

	
}

//将参数传给基类
Mpu9250_Ahrs::Mpu9250_Ahrs(I2c *i2c):Mpu9250(i2c)
{
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	Pitch_off = 0.00f;
	Roll_off = 0.00f;
	Yaw_off = 0.00f;

}

int Mpu9250_Ahrs::Get_MPU9250_Data(void)
{
	u8 id;
	u8 AK_id;
	int16_t temp[7];
	int16_t AK_temp[3];
	//数据读出
	this->mode(MPU6500);
	this->get_id(&id);
	if (id == 0x73)
		this->get_data(ACCEL_XOUT_H, temp, 7);
	else
		return -1;
	delay_ms(10);
	this->mode(AK8963);
	delay_ms(10);
	this->get_id(&AK_id);
	if (AK_id == 0x48)
		this->write_data(MAG_CNTL1, 0x11);
	else
		return -2;
	this->write_data(MAG_TEST1, 0x08);
	AK_temp[0] = this->get_data_2byte(RA_MAG_XOUT_L);
	AK_temp[1] = this->get_data_2byte(RA_MAG_YOUT_L);
	AK_temp[2] = this->get_data_2byte(RA_MAG_ZOUT_L);
	this->get_data_2byte(RA_MAG_ST2);

    //数据传入
	//加速度
	this->Accbuf.X = temp[0];
	this->Accbuf.Y = temp[1];
	this->Accbuf.Z = temp[2];
	//陀螺仪
	this->Gyrobuf.X = temp[4];
	this->Gyrobuf.Y = temp[5];
	this->Gyrobuf.Z = temp[6];
	//磁力计
	this->Magbuf.X = AK_temp[0];
	this->Magbuf.Y = AK_temp[1];
	this->Magbuf.Z = AK_temp[2];
	return 0;
}

void Mpu9250_Ahrs::get_data_buf(int16_t *mpu, int16_t *AK)
{
	*mpu++ = this->Accbuf.X;
	*mpu++ = this->Accbuf.Y;
	*mpu++ = this->Accbuf.Z;

	*mpu++ = this->Gyrobuf.X;
	*mpu++ = this->Gyrobuf.Y;
	*mpu   = this->Gyrobuf.Z;

	*AK++ = this->Magbuf.X;
	*AK++ = this->Magbuf.Y;
	*AK   = this->Magbuf.Z;

}

void Mpu9250_Ahrs::AHRS_Dataprepare()
{
	this->Get_MPU9250_Data();//先获取数据
	 //16.4 = 2^16/4000 lsb °/s     1/16.4=0.061     0.0174 = 3.14/180
	 //陀螺仪数据从ADC转化为弧度每秒(这里需要减去偏移值)
	GyroFinal.X = (Gyrobuf.X - Gyrooffset.X)*0.061*0.0174;
	GyroFinal.Y = (Gyrobuf.Y - Gyrooffset.Y)*0.061*0.0174;
	GyroFinal.Z = (Gyrobuf.Z - Gyrooffset.Z)*0.061*0.0174;		//读出值减去基准值乘以单位，计算陀螺仪角速度

    //+-2g,2^16/4=16384lsb/g--0.061mg/lsb
	//此处0.0098是：(9.8m/s^2)/1000,乘以mg得m/s^2
	AccFinal.X = (float)((Accbuf.X - Accoffset.X)*0.061)*0.0098;
	AccFinal.Y = (float)((Accbuf.Y - Accoffset.Y)*0.061)*0.0098;
	AccFinal.Z = (float)((Accbuf.Z - Accoffset.Z)*0.061)*0.0098;

	//±4800uT 2^16/9600 = 6.83lsb/uT     1/6.83 = 0.1465
	//地磁强度为 5-6 x 10^(-5) T = 50 - 60 uT
	MagFinal.X = (float)(Magbuf.X - Magoffset.X)*0.1465;
	MagFinal.Y = (float)(Magbuf.Y - Magoffset.Y)*0.1465;
	MagFinal.Z = (float)(Magbuf.Z - Magoffset.Z)*0.1465;
}

void Mpu9250_Ahrs::Acc_Correct()
{
	u8 i = 0;
	u8 numAcc = 200;//取200次累计量

	int Angleaccx = 0;  //加速度计校正中间变量
	int Angleaccy = 0;
	int Angleaccz = 0;

	for (i = 0; i<numAcc; i++)
	{
		Get_MPU9250_Data();
		Angleaccx += Accbuf.X;
		Angleaccy += Accbuf.Y;
		Angleaccz += Accbuf.Z;
		delay_ms(2);
	}
	Accoffset.X = Angleaccx / numAcc;
	Accoffset.Y = Angleaccy / numAcc;
	Accoffset.Z = Angleaccy / numAcc;				   //得到加速度计基准
}

void Mpu9250_Ahrs::Gyro_Correct()
{
	unsigned char i = 0;
	unsigned char numGyro = 200;

	int Gyrox = 0;
	int Gyroy = 0;
	int Gyroz = 0;							  //陀螺仪校正中间变量

	for (i = 0; i<numGyro; i++)
	{
		Get_MPU9250_Data();
		Gyrox += Gyrobuf.X;
		Gyroy += Gyrobuf.Y;
		Gyroz += Gyrobuf.Z;
		delay_ms(2);
	}

	Gyrooffset.X = Gyrox / numGyro;
	Gyrooffset.Y = Gyroy / numGyro;
	Gyrooffset.Z = Gyroz / numGyro;
}

void Mpu9250_Ahrs::Mag_Correct()
{
	unsigned char i = 0;
	unsigned char numMag = 200;
	int Magx = 0;
	int Magy = 0;
	int Magz = 0;							  //磁力计校正中间变量

	for (i = 0; i<numMag; i++)
	{
		Get_MPU9250_Data();
		Magx += Magbuf.X;
		Magy += Magbuf.Y;
		Magz += Magbuf.Z;
		delay_ms(2);
	}

	Magoffset.X = Magx / numMag;
	Magoffset.Y = Magy / numMag;
	Magoffset.Z = Magz / numMag;
}

void Mpu9250_Ahrs::get_data_adc(float *mpu, float *AK)
{
	*mpu++ = this->AccFinal.X;
	*mpu++ = this->AccFinal.Y;
	*mpu++ = this->AccFinal.Z;
	*mpu++ = this->GyroFinal.X;
	*mpu++ = this->GyroFinal.Y;
	*mpu =   this->GyroFinal.Z;

	*AK++ = this->MagFinal.X;
	*AK++ = this->MagFinal.Y;
	*AK =   this->MagFinal.Z;


}

float Mpu9250_Ahrs::invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void Mpu9250_Ahrs::AHRSupdate(void)
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
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	//float exInt = 0, eyInt = 0, ezInt = 0;//按比例缩小积分误差
	volatile float beta = betaDef;								// 2 * proportional gain (Kp)

	//四元数变化率
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		//正常化的加速度测量值
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		//正常化的磁力计测量值
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		//辅助变量，避免重复运算
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
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

		// Reference direction of Earth's magnetic field
		//地球磁场参考方向
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		//梯度校正算法
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		//应用反馈步骤
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	//四元数变化率的集成率
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// 正常化四元数
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;


	//四元数转换成欧拉角
	Pitch = asin(2 * q0*q2 - 2 * q1*q3) / 3.14 * 180;
	Roll = atan2(2 * q0q1 + 2 * q2q3, 1 - 2 * q1q1 - 2 * q2q2) / 3.14 * 180;
	Yaw = atan2(2 * q0q3 + 2 * q1*q2, 1 - 2 * q2*q2 - 2 * q3*q3) / 3.14 * 180;
}

void Mpu9250_Ahrs::get_data_ahrs(float *m_Pitch, float *m_Roll, float *m_Yaw)
{
	*m_Pitch = Pitch-Pitch_off;
	*m_Roll = Roll-Roll_off;
	*m_Yaw = Yaw-Yaw_off;
}

void Mpu9250_Ahrs::get_data_q(float *q)
{
	*q++ = q0;
	*q++ = q1;
	*q++ = q2;
	*q = q3;
}

void Mpu9250_Ahrs::update_data(void)
{
	this->Pitch_off = Pitch;
	this->Roll_off = Roll;
	this->Yaw_off = Yaw;
}