#include "imu.h"
#include "mpu9250.h"
#include "delay.h"

extern SENSOR_DATA Gyrobuf;//������
extern SENSOR_DATA Accbuf;//���ٶ�
extern SENSOR_DATA Magbuf;//������

SENSOR_DATA Accoffset;//���ٶ�ƫ����
SENSOR_DATA Gyrooffset;//������ƫ����
SENSOR_DATA Magoffset;//������ƫ����


//����������
extern IMU_DATA GyroFinal;
extern IMU_DATA AccFinal;
extern IMU_DATA MagFinal;

/*
#define Kp 2.0f	//��������֧��������accellrometer/magnetometer
#define Ki 0.005f //��������ִ�����������ǵ��ν�gyroscopeases
#define halfT 0.002f//�������ڵ�һ�룬������Ϊ10ms,��һ��Ϊ0.005s*/

float temp;//�¶�
extern float Pitch;
extern float Roll;
extern float Yaw;



#define sampleFreq	50.0f		// sample frequency in Hz  ������ 100 HZ  10ms  �޸Ĵ�Ƶ�ʿ����ӱ仯�ٶ�
#define betaDef		0.1f		// 2 * proportional gain


volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relativ


float exInt = 0, eyInt = 0,ezInt = 0;//��������С�������


//������ƽ����
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


//��ȡMPU9250����
void Get_MPU9250_DATA(void)
{
	READ_MPU9250_GYRO(&Gyrobuf.X,&Gyrobuf.Y,&Gyrobuf.Z);//��ȡ����������-ADC������
	READ_MPU9250_ACCEL(&Accbuf.X,&Accbuf.Y,&Accbuf.Z);//��ȡ���ٶȼ�����-ADC������
	READ_MPU9250_MAG(&Magbuf.X,&Magbuf.Y,&Magbuf.Z);//��ȡ����������-ADC������
	READ_MPU9250_TEMP(&temp);						  //��ȡ�¶�-�¶�ģ����
}


void AHRS_Dataprepare(void)
{
	Get_MPU9250_DATA();//�Ȼ�ȡ����
	
	//16.4 = 2^16/4000 lsb ��/s     1/16.4=0.061     0.0174 = 3.14/180
	//���������ݴ�ADCת��Ϊ����ÿ��(������Ҫ��ȥƫ��ֵ)
	GyroFinal.X=(Gyrobuf.X-Gyrooffset.X)*0.061*0.0174;
	GyroFinal.Y=(Gyrobuf.Y-Gyrooffset.Y)*0.061*0.0174;
	GyroFinal.Z=(Gyrobuf.Z-Gyrooffset.Z)*0.061*0.0174;		//����ֵ��ȥ��׼ֵ���Ե�λ�����������ǽ��ٶ�
	/*
	AccFinal.X=(float)(Accbuf.X-Accoffset.X);	
	AccFinal.Y=(float)(Accbuf.Y-Accoffset.Y);		
	AccFinal.Z=(float)(Accbuf.Z-Accoffset.Z); 
	*/
	//+-8g,2^16/16=4096lsb/g--0.244mg/lsb
	//�˴�0.0098�ǣ�(9.8m/s^2)/1000,����mg��m/s^2
	AccFinal.X=(float)((Accbuf.X-Accoffset.X)*0.244)*0.0098;		
	AccFinal.Y=(float)((Accbuf.Y-Accoffset.Y)*0.244)*0.0098;		
	AccFinal.Z=(float)((Accbuf.Z-Accoffset.Z)*0.244)*0.0098;
		
	
	//��4800uT 2^16/9600 = 6.83lsb/uT     1/6.83 = 0.1465
	//�ش�ǿ��Ϊ 5-6 x 10^(-5) T = 50 - 60 uT
	MagFinal.X = (float)(Magbuf.X-Magoffset.X)*0.1465;
	MagFinal.Y = (float)(Magbuf.Y-Magoffset.Y)*0.1465;
	MagFinal.Z = (float)(Magbuf.Z-Magoffset.Z)*0.1465;

}

//���ٶȼ�У��
void Acc_Correct(void)
{
	u8 i = 0;
	u8 numAcc = 200;//ȡ200���ۼ���
	
	int Angleaccx=0;  //���ٶȼ�У���м����
	int Angleaccy=0;
	int Angleaccz=0;							

	for(i=0;i<numAcc;i++)
	{		
		Get_MPU9250_DATA();
		Angleaccx+=Accbuf.X;
		Angleaccy+=Accbuf.Y;
		Angleaccz+=Accbuf.Z;
		delay_ms(2);
	}	
	Accoffset.X= Angleaccx/numAcc;					   
	Accoffset.Y= Angleaccy/numAcc;
	Accoffset.Z= Angleaccy/numAcc;				   //�õ����ٶȼƻ�׼
}

//������У��
void Gyro_Correct(void)
{
	unsigned char i=0;
	unsigned char numGyro=200;

	int Gyrox=0;
	int Gyroy=0;
	int Gyroz=0;							  //������У���м����

	for(i=0;i<numGyro;i++)
	{
		Get_MPU9250_DATA();
		Gyrox+=Gyrobuf.X;
		Gyroy+=Gyrobuf.Y;
		Gyroz+=Gyrobuf.Z;
		delay_ms(2);
	}
	
	Gyrooffset.X= Gyrox/numGyro;					   
	Gyrooffset.Y= Gyroy/numGyro;
	Gyrooffset.Z= Gyroz/numGyro;
}


//������У��
void Mag_Correct(void)
{
	unsigned char i=0;
	unsigned char numMag=200;
	int Magx=0;
	int Magy=0;
	int Magz=0;							  //������У���м����
	
	for(i=0;i<numMag;i++)
	{
		Get_MPU9250_DATA();
		Magx+=Magbuf.X;
		Magy+=Magbuf.Y;
		Magz+=Magbuf.Z;
		delay_ms(2);
	}
	
	Magoffset.X= Magx/numMag;					   
	Magoffset.Y= Magy/numMag;
	Magoffset.Z= Magz/numMag;
}


//9����̬����
void AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	//��������Ʋ�����Ч��ʹ��IMU(6��)�㷨
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))		
	{
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	//��Ԫ���仯��
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	//
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
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

		// Auxiliary variables to avoid repeated arithmetic
		//���������������ظ�����
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
		//����ų��ο�����
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		//�ݶ�У���㷨
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
		//Ӧ�÷�������
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	//��Ԫ���仯�ʵļ�����
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// ��������Ԫ��
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	
	//��Ԫ��ת����ŷ����
	Pitch = asin(2*q0*q2-2*q1*q3)/3.14*180;
	Roll = atan2(2*q0q1+2*q2q3,1-2*q1q1-2*q2q2)/3.14*180;
	Yaw = atan2(2*q0q3+2*q1*q2,1-2*q2*q2-2*q3*q3)/3.14*180;
		
}



// IMU algorithm update  6����̬����
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	float q0q1,q0q2,q0q3,q1q2,q1q3,q2q3;//�Լ���Ӳ���
	
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;
		
		//�Լ���Ӳ���
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q3 = q2 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	
	//��Ԫ��ת����ŷ����
	Pitch = asin(2*q0*q2-2*q1*q3)/3.14*180;
	Roll = atan2(2*q0q1+2*q2q3,1-2*q1q1-2*q2q2)/3.14*180;
	Yaw = atan2(2*q0q3+2*q1*q2,1-2*q2*q2-2*q3*q3)/3.14*180;
	
}



