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
//Mpu9250 mpu(&i2c1);
Mpu9250_Ahrs mpu(&i2c1);

void setup() 
{    
	ebox_init();
	uart1.begin(115200);
	mpu.begin(400000);
	PA8.mode(OUTPUT_PP);
}
	int16_t tmp[7];
	int16_t AK_tmp[3];
	int8_t AK_test[10];
	int16_t mag;
	int16_t x, y, z;
	uint8_t id;
	uint8_t AK_id;
	u8 cotrl1;
	int8_t ST1;
	u8 mpu6050_temp;
	u8 AK_temp;
	int main(void)
	{
		setup();

		while (1)
		{
			mpu.Get_MPU9250_Data();
			mpu.get_data_buf(tmp, AK_tmp);
			/*
			mpu.mode(MPU6500);
			mpu.get_id(&id);
			mpu.get_data(ACCEL_XOUT_H, tmp, 7);
			delay_ms(10);
			mpu.mode(AK8963);
			delay_ms(10);
			mpu.get_id(&AK_id);
			mpu.write_data(MAG_CNTL1, 0x11);
			mpu.write_data(MAG_TEST1, 0x08);
			//mpu.get_data(RA_MAG_XOUT_L, AK_tmp, 1);
			uart1.printf("\r\nmag = %d", id);
			uart1.printf("\r\nAK_ID = %d", AK_id);

			
			AK_tmp[0] = mpu.get_data_2byte(RA_MAG_XOUT_L);
			AK_tmp[1] = mpu.get_data_2byte(RA_MAG_YOUT_L);
			AK_tmp[2] = mpu.get_data_2byte(RA_MAG_ZOUT_L);
			mpu.get_data_2byte(RA_MAG_ST2);
			*/
			uart1.printf("\r\naccx = %d", tmp[0]);
			uart1.printf("\r\naccy = %d", tmp[1]);
			uart1.printf("\r\naccz = %d", tmp[2]);
			uart1.printf("\r\ntemp = %d", tmp[3]);
			uart1.printf("\r\ngyrox = %d", tmp[4]);
			uart1.printf("\r\ngyroy = %d", tmp[5]);
			uart1.printf("\r\ngyroz = %d", tmp[6]);
			uart1.printf("\r\n==========");


			uart1.printf("\r\ntest0 = %d", AK_tmp[0]);
			uart1.printf("\r\ntest1 = %d", AK_tmp[1]);
			uart1.printf("\r\ntest2 = %d", AK_tmp[2]);
		//	uart1.printf("\r\ntest3 = %d", AK_test[3]);

			delay_ms(100);
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



