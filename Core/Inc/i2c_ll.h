/*
 * i2c_ll.h
 *
 *  Created on: Jan 8, 2026
 *      Author: OkhrimenkoDI
 */

#ifndef INC_I2C_LL_H_
#define INC_I2C_LL_H_

#pragma once
#include <stdint.h>
#include <stdbool.h>

bool i2c_ll_write(uint8_t addr, uint8_t reg,
                  const uint8_t *data, uint8_t len);

bool i2c_ll_read (uint8_t addr, uint8_t reg,
                  uint8_t *data, uint8_t len);


#endif /* INC_I2C_LL_H_ */
