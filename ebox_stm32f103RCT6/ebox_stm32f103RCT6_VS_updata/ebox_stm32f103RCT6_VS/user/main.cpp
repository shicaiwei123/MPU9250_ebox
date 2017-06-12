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
#include "mpu9250.h"
#include "uart_vcan.h"
//Mpu9250<SoftI2c> mpu(&i2c1);
Mpu9250Ahrs<I2c>  mpu(&i2c1);
UartVscan uartv(&uart1);
void setup() 
{    
	ebox_init();
//	uartv.begin(115200);   //山外调试助手
	mpu.begin(400000);
	uart1.begin(115200);
	mpu.setParameter(10.3f, 0.008, 125);
	PA8.mode(OUTPUT_PP);
	uart1.printf("intial....");
	mpu.accCorrect();
	mpu.gyroCorrect();
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
	float q[4];
	float pitch;
	float roll;
	float yaw;
	int main(void)
	{
		setup();
		int i=0;
		uint64_t time;
		while (1)
		{
			time = millis();
			i++;
			mpu.getDataAhrs(&pitch, &roll, &yaw);
			/*
		//关于四元素的测试输出
		//	uart1.printf("\r\ntest3 = %d", AK_test[3]);
			uart1.printf("\r\nq0 = %.1f", q[0]);
			uart1.printf("\r\nq1 = %.1f", q[1]);
			uart1.printf("\r\nq2 = %.1f", q[2]);
			uart1.printf("\r\nq3 = %.1f", q[3]);
			*/
			///*
			if (i == 3)
			{
				//关于原始数据的测试输出
				/*
				uart1.printf("\r\naccx = %d", tmp[0]);
				uart1.printf("\r\naccy = %d", tmp[1]);
				uart1.printf("\r\naccz = %d", tmp[2]);
				uart1.printf("\r\ntemp = %d", tmp[3]);
				uart1.printf("\r\ngyrox = %d", tmp[4]);
				uart1.printf("\r\ngyroy = %d", tmp[5]);
				uart1.printf("\r\ngyroz = %d", tmp[6]);
				uart1.printf("\r\n==========");
				uart1.printf("\r\nmx = %d", AK_tmp[0]);
				uart1.printf("\r\nmy = %d", AK_tmp[1]);
				uart1.printf("\r\nmz= %d", AK_tmp[2]);
				uart1.printf("\r\n==========");
				*/
				uart1.printf("\r\npitch = %.2f", pitch);
				uart1.printf("\r\nroll = %.2f", roll);
				uart1.printf("\r\nyaw = %.2f", yaw);
				//uart1.printf("\r\n pitch=%.2f", pitch);
				//uart1.printf("\r\n roll=%.2f", roll);
				//uartv.sendOscilloscope(pitch);
				//uartv.sendOscilloscope(roll);
				i = 0;
			}
		//	*/
			
			//delay_ms(100);
			delay_us(4500);
			//uart1.printf("fps:%.1f\t", 1.0 / (millis() - time) * 1000);
			PA8.toggle();//将当前的状态进行翻转 
 
		}
  } 



