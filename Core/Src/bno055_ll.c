/*
 * bno055_ll.c
 *
 *  Created on: Jan 8, 2026
 *      Author: OkhrimenkoDI
 */


#include "bno055_ll.h"
#include "i2c_ll.h"

static uint8_t bno_addr = BNO055_ADDR_LOW;
static uint8_t current_page = 0xFF;

bool bno055_ll_set_page(uint8_t page)
{
    if (page == current_page)
        return true;

    if (!i2c_ll_write(bno_addr, BNO055_PAGE_ID, &page, 1))
        return false;

    current_page = page;
    return true;
}

bool bno055_ll_read_reg(uint8_t reg, uint8_t *val)
{
    if (!bno055_ll_set_page(0))
        return false;

    return i2c_ll_read(bno_addr, reg, val, 1);
}

bool bno055_ll_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!bno055_ll_set_page(0))
        return false;

    return i2c_ll_read(bno_addr, reg, buf, len);
}

bool bno055_ll_write_reg(uint8_t reg, uint8_t val)
{
    if (!bno055_ll_set_page(0))
        return false;

    return i2c_ll_write(bno_addr, reg, &val, 1);
}
