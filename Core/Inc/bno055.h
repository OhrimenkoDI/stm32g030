/*
 * bno055.h
 *
 *  Created on: Jan 8, 2026
 *      Author: OkhrimenkoDI
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* I2C address */
// Адреса BNO-055
#define BNO055_ADDR 0x29 // Адрес при низком уровне на выводе COM3 ADD
#define BNO055_ADDR_B 0x29 // Адрес при высоком уровне на выводе COM3 ADD


/* CHIP ID */
#define BNO055_ID          0xA0

/* Registers */
#define BNO055_CHIP_ID_ADDR            0x00
#define BNO055_PAGE_ID_ADDR            0x07
#define BNO055_ACCEL_DATA_X_LSB_ADDR   0x08
#define BNO055_MAG_DATA_X_LSB_ADDR     0x0E
#define BNO055_GYRO_DATA_X_LSB_ADDR    0x14
#define BNO055_EULER_H_LSB_ADDR        0x1A
#define BNO055_QUATERNION_DATA_W_LSB   0x20
#define BNO055_LINEAR_ACCEL_DATA_X_LSB 0x28
#define BNO055_GRAVITY_DATA_X_LSB      0x2E
#define BNO055_TEMP_ADDR               0x34
#define BNO055_CALIB_STAT_ADDR         0x35
#define BNO055_UNIT_SEL_ADDR           0x3B
#define BNO055_OPR_MODE_ADDR           0x3D
#define BNO055_SYS_TRIGGER_ADDR        0x3F
#define BNO055_AXIS_MAP_CONFIG_ADDR    0x41
#define BNO055_AXIS_MAP_SIGN_ADDR      0x42

/* Modes */
#define OPERATION_MODE_CONFIG  0x00
#define OPERATION_MODE_NDOF    0x0C

/* API */
bool bno055_init(void);

bool bno055_read_calibration(uint8_t *sys, uint8_t *gyro,
                             uint8_t *accel, uint8_t *mag);

bool bno055_read_euler(float *heading, float *roll, float *pitch);
bool bno055_read_quaternion(float *qw, float *qx, float *qy, float *qz);
bool bno055_read_accel(float *ax, float *ay, float *az);
bool bno055_read_gyro(float *gx, float *gy, float *gz);
bool bno055_read_mag(float *mx, float *my, float *mz);
bool bno055_read_linear_accel(float *ax, float *ay, float *az);
bool bno055_read_gravity(float *gx, float *gy, float *gz);


#endif /* INC_BNO055_H_ */
