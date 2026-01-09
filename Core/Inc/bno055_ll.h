/*
 * bno055_ll.h
 *
 *  Created on: Jan 8, 2026
 *      Author: OkhrimenkoDI
 */

#ifndef INC_BNO055_LL_H_
#define INC_BNO055_LL_H_

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* I2C address */
#define BNO055_ADDR_LOW   0x28
#define BNO055_ADDR_HIGH  0x29

/* Registers */
#define BNO055_CHIP_ID        0x00
#define BNO055_PAGE_ID        0x07
#define BNO055_OPR_MODE       0x3D
#define BNO055_SYS_TRIGGER    0x3F
#define BNO055_UNIT_SEL       0x3B

#define BNO055_EULER_H_LSB    0x1A
#define BNO055_QUAT_W_LSB     0x20
#define BNO055_CALIB_STAT    0x35

/* Modes */
#define BNO055_MODE_CONFIG    0x00
#define BNO055_MODE_NDOF      0x0C

bool bno055_ll_read_reg (uint8_t reg, uint8_t *val);
bool bno055_ll_read_regs(uint8_t reg, uint8_t *buf, uint8_t len);
bool bno055_ll_write_reg(uint8_t reg, uint8_t val);
bool bno055_ll_set_page (uint8_t page);




#endif /* INC_BNO055_LL_H_ */
