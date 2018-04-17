/**
 * Copyright (C) 2017 Sergio J. Munoz Lopez <semulopez@gmail.com>
 * 
 *    Tutor: Corrado Guarino Lo Bianco <guarino@ce.unipr.it>  
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

#ifndef _I2C_RT_
#define _I2C_RT_

/* FLAGS OPTIONS (all deactivated by default) */

#define I2C_RT_FLAG_READ_REPEATED_START		1
#define I2C_RT_FLAG_WRITE_REPEATED_START	2
#define I2C_RT_FLAG_DEBUG_MODE			4
#define I2C_RT_FLAG_RECONFIGURE_R_W		8


int I2C_RT_CLOSE(int *);
int I2C_RT_OPEN(int *, uint8_t);
int I2C_RT_OPEN_W_FLAGS(int *, uint8_t, int);
int I2C_RT_SET_SLAVE(int, uint8_t);
int I2C_RT_SET_FLAGS(int, int);
int I2C_RT_SET_BAUDRATE(int, int);
int I2C_RT_SET_CLOCK_DIVIDER(int, int);

int I2C_RT_READ_RAW(int, char*, int);
int I2C_RT_READ_BYTE(int, char);
int I2C_RT_READ_WORD(int, char);
int I2C_RT_READ_BLOCK(int, char);

int I2C_RT_WRITE_RAW(int, char*, int);
int I2C_RT_WRITE_BYTE(int, char, char);
int I2C_RT_WRITE_WORD(int, char, uint16_t);
int I2C_RT_WRITE_BLOCK(int, char, int);

int I2C_RT_IS_CONNECTED_WORD(int device_handle, char reg_address, uint16_t value);

#endif /* _I2C_RT_ */

