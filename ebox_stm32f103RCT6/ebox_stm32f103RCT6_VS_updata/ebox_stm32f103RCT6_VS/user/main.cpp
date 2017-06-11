/**
  ******************************************************************************
  * @file   : *.cpp
  * @author : shentq
  * @version: V1.2
  * @date   : 2016/08/14

  * @brief   ebox application example .
  *
  * Copyright 2016 shentq. All Rights Reserved.         
  ******************************************************************************
 */


#include "ebox.h" 
#include "MPU6050.h"
#include "uart_vcan.h"
//Mpu9250 mpu(&i2c1);
Mpu9250_Ahrs mpu(&si2c1);
UartVscan uartv(&uart1);
void setup() 
{    
	ebox_init();
	uart1.begin(115200);
	uartv.begin(115200);
	mpu.set_parameter(10.3f, 0.008, 125);
	mpu.begin(400000);
	PA8.mode(OUTPUT_PP);
	//uart1.printf("intial....");
	mpu.acc_correct();
	mpu.gyro_correct();
//	mpu.Mag_Correct();
	
}
	int16_t tmp[7];
	int16_t AK_tmp[3];
	float tmp_adc[7];
	float AK_tmp_adc[3];
	int8_t AK_test[10];
	int16_t mag;
	int16_t x, y, z;
	uint8_t id;
	uint8_t AK_id;
	u8 cotrl1;
	int8_t ST1;
	u8 mpu6050_temp;
	u8 AK_temp;
	float q[4];
	float pitch;
	float roll;
	float yaw;
	u16 count;
	int main(void)
	{
		setup();
		count = 0;
		int i=0;
		uint64_t time;
		float ki, kp, sample, halfT;
		uint8_t  sampleH;
		while (1)
		{
			time = millis();
			i++;
			count++;
			//if (count == 100)
			//	mpu.update_data();
			mpu.ahrs_dataprepare();
			mpu.get_data_buf(tmp, AK_tmp);
			//mpu.get_data_adc(tmp_adc, AK_tmp_adc);
			//mpu.get_data_q(q);
		    mpu.ahrs_update();
			mpu.get_data_ahrs(&pitch, &roll, &yaw);
			mpu.get_parameter(&kp, &ki, &sample, &sampleH, &halfT);
			

			
			/*
			uart1.printf("\r\nkp = %.5f", kp);
			uart1.printf("\r\nki = %.5f", ki);
			uart1.printf("\r\nsample = %.1f", sample);
			uart1.printf("\r\nsampleH = %x", sampleH);
			uart1.printf("\r\nhalfT = %.5f", halfT);
			uart1.printf("\r\n==========");
			*/
			/*
		//	uart1.printf("\r\ntest3 = %d", AK_test[3]);
			uart1.printf("\r\nq0 = %.1f", q[0]);
			uart1.printf("\r\nq1 = %.1f", q[1]);
			uart1.printf("\r\nq2 = %.1f", q[2]);
			uart1.printf("\r\nq3 = %.1f", q[3]);
			*/
			///*
			if (i == 3)
			{
				/*
				uart1.printf("\r\naccx = %d", tmp[0]);
				uart1.printf("\r\naccy = %d", tmp[1]);
				uart1.printf("\r\naccz = %d", tmp[2]);
				uart1.printf("\r\ntemp = %d", tmp[3]);
				uart1.printf("\r\ngyrox = %d", tmp[4]);
				uart1.printf("\r\ngyroy = %d", tmp[5]);
				uart1.printf("\r\ngyroz = %d", tmp[6]);
				uart1.printf("\r\n==========");

				*/
				uart1.printf("\r\ntest0 = %d", AK_tmp[0]);
				uart1.printf("\r\ntest1 = %d", AK_tmp[1]);
				uart1.printf("\r\ntest2 = %d", AK_tmp[2]);
				uart1.printf("\r\n==========");
				uart1.printf("\r\npitch = %.2f", pitch);
				uart1.printf("\r\nroll = %.2f", roll);
				uart1.printf("\r\nyaw = %.2f", yaw);
				uart1.printf("\r\n pitch=%.2f", pitch);
				uart1.printf("\r\n roll=%.2f", roll);
				//uartv.sendOscilloscope(pitch);
				//uartv.sendOscilloscope(roll);
				//printf("%.2f\r\n", mpu.invSqrt(0.25f));
				i = 0;
			}
		//	*/
			
			//delay_ms(100);
			delay_us(4500);
			//uart1.printf("fps:%.1f\t", 1.0 / (millis() - time) * 1000);
			PA8.toggle();//将当前的状态进行翻转 
			/*
			AK_test[0] = mpu.get_data_2byte(RA_MAG_XOUT_L);
			AK_test[1] = mpu.get_data_2byte(RA_MAG_YOUT_L);
			AK_test[2] = mpu.get_data_2byte(RA_MAG_ZOUT_L);
			AK_test[3] = mpu.get_data_2byte(RA_MAG_ST1);
			AK_test[4] = mpu.get_data_2byte(RA_MAG_ST2);

		*/
 
		}
  } 



