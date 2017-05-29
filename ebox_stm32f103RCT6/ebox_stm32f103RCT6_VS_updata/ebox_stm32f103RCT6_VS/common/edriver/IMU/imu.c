#include "imu.h"
#include "mpu9250.h"
#include "delay.h"

extern SENSOR_DATA Gyrobuf;//陀螺仪
extern SENSOR_DATA Accbuf;//加速度
extern SENSOR_DATA Magbuf;//磁力计

SENSOR_DATA Accoffset;//加速度偏移量
SENSOR_DATA Gyrooffset;//陀螺仪偏移量
SENSOR_DATA Magoffset;//磁力计偏移量


//处理后的数据
extern IMU_DATA GyroFinal;
extern IMU_DATA AccFinal;
extern IMU_DATA MagFinal;

/*
#define Kp 2.0f	//比例增益支配收敛率accellrometer/magnetometer
#define Ki 0.005f //积分增益执行速率陀螺仪的衔接gyroscopeases
#define halfT 0.002f//采样周期的一半，若周期为10ms,则一般为0.005s*/

float temp;//温度
extern float Pitch;
extern float Roll;
extern float Yaw;



#define sampleFreq	50.0f		// sample frequency in Hz  采样率 100 HZ  10ms  修改此频率可增加变化速度
#define betaDef		0.1f		// 2 * proportional gain


volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relativ


float exInt = 0, eyInt = 0,ezInt = 0;//按比例缩小积分误差


//快速逆平方根
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


//获取MPU9250数据
void Get_MPU9250_DATA(void)
{
	READ_MPU9250_GYRO(&Gyrobuf.X,&Gyrobuf.Y,&Gyrobuf.Z);//读取陀螺仪数据-ADC数字量
	READ_MPU9250_ACCEL(&Accbuf.X,&Accbuf.Y,&Accbuf.Z);//读取加速度计数据-ADC数字量
	READ_MPU9250_MAG(&Magbuf.X,&Magbuf.Y,&Magbuf.Z);//读取磁力计数据-ADC数字量
	READ_MPU9250_TEMP(&temp);						  //读取温度-温度模拟量
}


void AHRS_Dataprepare(void)
{
	Get_MPU9250_DATA();//先获取数据
	
	//16.4 = 2^16/4000 lsb °/s     1/16.4=0.061     0.0174 = 3.14/180
	//陀螺仪数据从ADC转化为弧度每秒(这里需要减去偏移值)
	GyroFinal.X=(Gyrobuf.X-Gyrooffset.X)*0.061*0.0174;
	GyroFinal.Y=(Gyrobuf.Y-Gyrooffset.Y)*0.061*0.0174;
	GyroFinal.Z=(Gyrobuf.Z-Gyrooffset.Z)*0.061*0.0174;		//读出值减去基准值乘以单位，计算陀螺仪角速度
	/*
	AccFinal.X=(float)(Accbuf.X-Accoffset.X);	
	AccFinal.Y=(float)(Accbuf.Y-Accoffset.Y);		
	AccFinal.Z=(float)(Accbuf.Z-Accoffset.Z); 
	*/
	//+-8g,2^16/16=4096lsb/g--0.244mg/lsb
	//此处0.0098是：(9.8m/s^2)/1000,乘以mg得m/s^2
	AccFinal.X=(float)((Accbuf.X-Accoffset.X)*0.244)*0.0098;		
	AccFinal.Y=(float)((Accbuf.Y-Accoffset.Y)*0.244)*0.0098;		
	AccFinal.Z=(float)((Accbuf.Z-Accoffset.Z)*0.244)*0.0098;
		
	
	//±4800uT 2^16/9600 = 6.83lsb/uT     1/6.83 = 0.1465
	//地磁强度为 5-6 x 10^(-5) T = 50 - 60 uT
	MagFinal.X = (float)(Magbuf.X-Magoffset.X)*0.1465;
	MagFinal.Y = (float)(Magbuf.Y-Magoffset.Y)*0.1465;
	MagFinal.Z = (float)(Magbuf.Z-Magoffset.Z)*0.1465;

}

//加速度计校正
void Acc_Correct(void)
{
	u8 i = 0;
	u8 numAcc = 200;//取200次累计量
	
	int Angleaccx=0;  //加速度计校正中间变量
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
	Accoffset.Z= Angleaccy/numAcc;				   //得到加速度计基准
}

//陀螺仪校正
void Gyro_Correct(void)
{
	unsigned char i=0;
	unsigned char numGyro=200;

	int Gyrox=0;
	int Gyroy=0;
	int Gyroz=0;							  //陀螺仪校正中间变量

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


//磁力计校正
void Mag_Correct(void)
{
	unsigned char i=0;
	unsigned char numMag=200;
	int Magx=0;
	int Magy=0;
	int Magz=0;							  //陀螺仪校正中间变量
	
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


//9轴姿态解算
void AHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	//如果磁力计测量无效则使用IMU(6轴)算法
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))		
	{
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	//四元数变化率
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	//
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
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
	Pitch = asin(2*q0*q2-2*q1*q3)/3.14*180;
	Roll = atan2(2*q0q1+2*q2q3,1-2*q1q1-2*q2q2)/3.14*180;
	Yaw = atan2(2*q0q3+2*q1*q2,1-2*q2*q2-2*q3*q3)/3.14*180;
		
}



// IMU algorithm update  6轴姿态解算
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	float q0q1,q0q2,q0q3,q1q2,q1q3,q2q3;//自己添加部分
	
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
		
		//自己添加部分
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
	
	
	//四元数转换成欧拉角
	Pitch = asin(2*q0*q2-2*q1*q3)/3.14*180;
	Roll = atan2(2*q0q1+2*q2q3,1-2*q1q1-2*q2q2)/3.14*180;
	Yaw = atan2(2*q0q3+2*q1*q2,1-2*q2*q2-2*q3*q3)/3.14*180;
	
}



