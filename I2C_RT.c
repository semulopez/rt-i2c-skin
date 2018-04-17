/**
 * Copyright (C) 2017 Sergio J. Munoz Lopez <semulopez@gmail.com>
 * 
 *    Tutor: Corrado Guarino Lo Bianco <guarino@ce.unipr.it> 
 *
 *    Version: 1.1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
 
/* RTDM header */
#include <rtdm/rtdm.h>

/* BCM283x I2C RTDM driver header */
#include <rtdm/i2c-bcm283x-rtdm.h>

/* Self-header */
#include "I2C_RT.h"


/**
 * Open i2c device and asign the slave address
 * @return 0 in case of success, a negative value otherwise.
 */

int I2C_RT_OPEN(int * device_handle, uint8_t slave_address) {

    int res;
    int value;

    /* Open device */
    res = rt_dev_open("/dev/rtdm/i2cdev0.0", 0);
    if (res < 0) {
        rt_printf("%s: Could not open i2c device, open has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        return -1;
    } else {
        rt_printf("%s: Device opened.\n", __FUNCTION__);
        *device_handle = res;
    }

    res = I2C_RT_SET_SLAVE(*device_handle, slave_address);

    return res;

}

/**
 * Open i2c device, asign the slave address and insert the flags options
 * @return 0 in case of success, a negative value otherwise.
 */

int I2C_RT_OPEN_W_FLAGS(int * device_handle, uint8_t slave_address, int flags) {

    int res;
    int value;

    /* Open device */
    res = rt_dev_open("/dev/rtdm/i2cdev0.0", flags);
    if (res < 0) {
        rt_printf("%s: Could not open i2c device, open has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        return -1;
    } else {
        rt_printf("%s: Device opened.\n", __FUNCTION__);
        *device_handle = res;
    }

    res = I2C_RT_SET_SLAVE(*device_handle, slave_address);

    return res;

}

/**
 * Close i2c device
 * @param[in, out] device fd pointer 
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_CLOSE(int *device_handle) {
	int res;

    res = rt_dev_close(*device_handle);
    if (res < 0) {
        rt_printf("%s: Could not close i2c device, close has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        return -1;
    } else {
        rt_printf("%s: Device closed.\n", __FUNCTION__);
		*device_handle = -1;
    }

	return 0;
}

/**
 * Set slave address
 * @param[in, out] device fd pointer 
 * @param[in] 'uint8_t' value of slave address
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_SET_SLAVE(int device_handle, uint8_t slave_address){

    int res = rt_dev_ioctl(device_handle, BCM283X_I2C_SET_SLAVE_ADDRESS, &slave_address);

    if (res < 0) {
        rt_printf("%s: Could not set slave address, ioctl has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        return -1;
    }

    return res;
}

/**
 * Set flags
 * @param[in] device fd pointer 
 * @param[in] flags (bit [0] -> READ REPEATED START | bit [1] -> WRITE REPEATED START | bit [2] -> DEBUG MODE | bit [3] -> RECONFIGURE DEVICE EACH WRITE/READ)
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_SET_FLAGS(int device_handle, int value){

    int res = rt_dev_ioctl(device_handle, BCM283X_I2C_SET_FLAGS, &value);

    if (res < 0) {
        rt_printf("%s: Could not set flags, ioctl has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        return -1;
    }

    return res;
}

/**
 * Set baudrate
 * @param[in] device fd pointer 
 * @param[in] value 
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_SET_BAUDRATE(int device_handle, int value){

    int res = rt_dev_ioctl(device_handle, BCM283X_I2C_SET_BAUDRATE, &value);

    if (res < 0) {
        rt_printf("%s: Could not set baudrate, ioctl has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        return -1;
    }

    return res;
}

/**
 * Set clock divider
 * @param[in] device fd pointer 
 * @param[in] value 
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_SET_CLOCK_DIVIDER(int device_handle, int value){

    int res = rt_dev_ioctl(device_handle, BCM283X_I2C_SET_CLOCK_DIVIDER, &value);

    if (res < 0) {
        rt_printf("%s: Could not set clock divider, ioctl has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        return -1;
    }

    return res;
}

/**
 * Read raw from i2c device
 * @param[in] device fd pointer 
 * @param[out] incoming buffer 
 * @param[in] size to be readed 
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_READ_RAW(int device_handle, char * buf, int size){

	int res;
	
	res = rt_dev_read(device_handle, (void *)buf, size);
	
	return res;

}

/**
 * Read one byte from i2c device
 * @param[in] device fd pointer 
 * @param[in] slave register
 * @return the readed value.
 */
int I2C_RT_READ_BYTE(int device_handle, char reg_address){

	int res;
	char buf;
	
	res = rt_dev_write(device_handle, (const void *)&reg_address, 1);
	
	/* Errors check */
	if (res != 0){
		rt_printf("%s: Error write\n", __FUNCTION__);
		return res;
	}
		
	res = rt_dev_read(device_handle, (void *)&buf, 1);
	
	/* Errors check */
	if (res < 0){
		rt_printf("%s: Error read\n", __FUNCTION__);
		return res;
	}
			
	return (int) buf;

}

/**
 * Read two bytes from i2c device
 * @param[in] device fd pointer 
 * @param[in] slave register address
 * @return the readed value.
 */
int I2C_RT_READ_WORD(int device_handle, char reg_address){
	
	int res;
	char buf [2] = {};

	res = rt_dev_write(device_handle, (const void *)&reg_address, 1);
	
	/* Errors check */
	if (res != 0){
		rt_printf("%s: Error write\n", __FUNCTION__);
		return res;
	}
		
	res = rt_dev_read(device_handle, (void *)buf, 2);
	
	/* Errors check */
	if (res < 0){
		rt_printf("%s: Error read\n", __FUNCTION__);
		return res;
	}
	
	return (int) (buf[0] | ((int)buf[1]) << 8);

}

/**
 * Read four bytes from i2c device
 * @param[in] device fd pointer 
 * @param[in] slave register address
 * @return the readed value.
 */
int I2C_RT_READ_BLOCK(int device_handle, char reg_address){

	int res;
	char buf [4];

	res = rt_dev_write(device_handle, (const void *)&reg_address, 1);
	
	/* Errors check */
	if (res != 0){
		rt_printf("%s: Error write\n", __FUNCTION__);
		return res;
	}
	
	res = rt_dev_read(device_handle, (void *)buf, 4);
	
	/* Errors check */
	if (res < 0){
		rt_printf("%s: Error read\n", __FUNCTION__);
		return res;
	}
	
	return (int) (buf[0] | ((int)buf[1]) << 8 | (buf[2] | ((int)buf[3]) << 8) << 16);

}

/**
 * Write raw to i2c device
 * @param[in] device fd pointer 
 * @param[in] outcoming buffer
 * @param[in] size of the buffer
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_WRITE_RAW(int device_handle, char * value, int size){
	
	int res;

	res = rt_dev_write(device_handle, (const void *)value, size);
	
	return res;

}

/**
 * Write one byte to i2c device
 * @param[in] device fd pointer  
 * @param[in] slave register address
 * @param[in] value to be writted
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_WRITE_BYTE(int device_handle, char reg_address, char value){
	
	int res;
	char buf[2] = {reg_address, value};
	
	res = rt_dev_write(device_handle, (const void *)buf, 2);
	
	/* Errors check */
	if (res != 0)
		rt_printf("%s: Error\n", __FUNCTION__);
		
	return res;

}

/**
 * Write two bytes to i2c device
 * @param[in] device fd pointer  
 * @param[in] slave register address
 * @param[in] value to be writted
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_WRITE_WORD(int device_handle, char reg_address, uint16_t value){
	
	int res;
	char buf[3] = {reg_address, (char)(value&0xFF) ,(char)((value&0xFF00)>>8)};

	res = rt_dev_write(device_handle, (const void *)buf, 3);
	
	/* Errors check */
	if (res != 0)
		rt_printf("%s: Error\n", __FUNCTION__);
		
	return res;

}

/**
 * Write four bytes to i2c device
 * @param[in] device fd pointer  
 * @param[in] slave register address
 * @param[in] value to be writted
 * @return 0 in case of success, a negative value otherwise.
 */
int I2C_RT_WRITE_BLOCK(int device_handle, char reg_address, int value){

	int res;
	char buf[5] = {reg_address, (char)(value&0xFF), (char)((value&0xFF00)>>8), (char)((value&0xFF0000)>>16), (char)((value&0xFF000000)>>24)};

	res = rt_dev_write(device_handle, (const void *)buf, 5);
	
	/* Errors check */
	if (res != 0)
		rt_printf("%s: Error\n", __FUNCTION__);
	
	return res;
	
}

int I2C_RT_IS_CONNECTED_WORD(int device_handle, char reg_address, uint16_t value){
	I2C_RT_WRITE_WORD(device_handle,reg_address,value);

	return (I2C_RT_READ_WORD(device_handle,reg_address)==value)?1:0;
}

