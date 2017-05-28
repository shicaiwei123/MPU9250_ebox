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


void Mpu9250::begin(uint32_t speed)
{
    this->speed = speed;
    i2c->take_i2c_right(this->speed);
    i2c->begin(this->speed);
    i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0);
    i2c->write_byte(SLAVEADDRESS, SMPLRT_DIV, 0x07);
    i2c->write_byte(SLAVEADDRESS, CONFIG, 0x06);
    i2c->write_byte(SLAVEADDRESS, GYRO_CONFIG, 0x18);
    i2c->write_byte(SLAVEADDRESS, ACCEL_CONFIG, 0x01);
    i2c->write_byte(SLAVEADDRESS, PWR_MGMT_1, 0);
    i2c->write_byte(SLAVEADDRESS, SMPLRT_DIV, 0x07);
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