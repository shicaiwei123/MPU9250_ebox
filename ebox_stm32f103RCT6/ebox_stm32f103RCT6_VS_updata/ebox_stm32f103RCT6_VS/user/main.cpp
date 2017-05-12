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
Mpu6050 mpu(&i2c1);
void setup() 
{    
	ebox_init();
	uart1.begin(115200);
	mpu.begin(400000);
	PA8.mode(OUTPUT_PP);
}
	int16_t tmp[7];
	int16_t AK_tmp[3];
	int16_t x, y, z;
	uint8_t id;
	uint8_t AK_id;
	int main(void)
	{
		setup();

		while (1)
		{
			mpu.get_id(&id);
			mpu.get_data(ACCEL_XOUT_H, tmp, 7);
			x = mpu.get_data(ACCEL_XOUT_H);
			y = mpu.get_data(ACCEL_YOUT_H);
			z = mpu.get_data(ACCEL_ZOUT_H);
			delay_ms(10);
			mpu.AK8963(0x02);
			delay_ms(5);
			mpu.getAK_id(&AK_id);
			mpu.AK8963(0x02);
			delay_ms(5);
			AK_tmp[0] = mpu.get_data_AK(RA_MAG_XOUT_L);
			mpu.AK8963(0x02);
			delay_ms(5);
			AK_tmp[1] = mpu.get_data_AK(RA_MAG_YOUT_L);
			mpu.AK8963(0x02);
			delay_ms(5);
			AK_tmp[2] = mpu.get_data_AK(RA_MAG_ZOUT_L);
		//	delay_ms(5);
		
			mpu.AK8963(0x00);
			uart1.printf("\r\nAKid = %d", AK_id);
			
			uart1.printf("\r\nid = %d", id);
			uart1.printf("\r\naccx = %d", tmp[0]);
			uart1.printf("\r\naccy = %d", tmp[1]);
			uart1.printf("\r\naccz = %d", tmp[2]);
			uart1.printf("\r\ntemp = %d", tmp[3]);
			uart1.printf("\r\ngyrox = %d", tmp[4]);
			uart1.printf("\r\ngyroy = %d", tmp[5]);
			uart1.printf("\r\ngyroz = %d", tmp[6]);
			uart1.printf("\r\n==========");
			uart1.printf("\r\nmagx = %d", AK_tmp[0]);
			uart1.printf("\r\namagy = %d", AK_tmp[1]);
			uart1.printf("\r\nmagz = %d", AK_tmp[2]);
			uart1.printf("\r\n==========");
			
			delay_ms(100);
			PA8.toggle();//将当前的状态进行翻转  
		}
  } 





/*
#include "ebox.h" 
TIM timer2(TIM2);//创建一个定时器中断对象，使用 TIM2。 
uint32_t count;
void t2_event()//定时器溢出中断事件 
{     count++;   
if(count == 1000)   
{         count = 0;      
PB8.toggle();     
uart1.printf("\r\ntimer2 is triggered 1000 times !");     
}
} 
void setup() 
{     ebox_init();
uart1.begin(115200); 
timer2.begin(1000);//初始化定时中断频率:1KHz    
timer2.attach_interrupt(t2_event);//绑定中断事件  
timer2.interrupt(ENABLE);//使能中断   
timer2.start();//启动定时器    
PB8.mode(OUTPUT_PP);
} 
*/