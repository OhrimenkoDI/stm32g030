#include "i2c_ll.h"
#include "stm32g0xx_ll_i2c.h"

#define I2C_TIMEOUT  100000UL

static bool i2c_wait_flag(volatile uint32_t (*flag)(I2C_TypeDef*),
                          I2C_TypeDef *i2c,
                          uint32_t expected)
{
    uint32_t t = I2C_TIMEOUT;
    while (flag(i2c) != expected)
    {
        if (--t == 0) return false;
    }
    return true;
}


bool i2c_ll_write(uint8_t addr, uint8_t reg,
                  const uint8_t *data, uint8_t len)
{
    if (LL_I2C_IsActiveFlag_BUSY(I2C1))
        return false;

    /* START + addr + (reg + data) */
    LL_I2C_HandleTransfer(I2C1,
                          addr << 1,
                          LL_I2C_ADDRSLAVE_7BIT,
                          len + 1,
                          LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_WRITE);

    /* Register */
    if (!i2c_wait_flag(LL_I2C_IsActiveFlag_TXIS, I2C1, 1))
        return false;
    LL_I2C_TransmitData8(I2C1, reg);

    /* Data */
    for (uint8_t i = 0; i < len; i++)
    {
        if (!i2c_wait_flag(LL_I2C_IsActiveFlag_TXIS, I2C1, 1))
            return false;
        LL_I2C_TransmitData8(I2C1, data[i]);
    }

    /* STOP */
    if (!i2c_wait_flag(LL_I2C_IsActiveFlag_STOP, I2C1, 1))
        return false;

    LL_I2C_ClearFlag_STOP(I2C1);
    return true;
}


bool i2c_ll_read(uint8_t addr, uint8_t reg,
                 uint8_t *data, uint8_t len)
{
    if (LL_I2C_IsActiveFlag_BUSY(I2C1))
        return false;

    /* Write register address */
    LL_I2C_HandleTransfer(I2C1,
                          addr << 1,
                          LL_I2C_ADDRSLAVE_7BIT,
                          1,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);

    if (!i2c_wait_flag(LL_I2C_IsActiveFlag_TXIS, I2C1, 1))
        return false;

    LL_I2C_TransmitData8(I2C1, reg);

    if (!i2c_wait_flag(LL_I2C_IsActiveFlag_TC, I2C1, 1))
        return false;

    /* ReSTART + Read */
    LL_I2C_HandleTransfer(I2C1,
                          addr << 1,
                          LL_I2C_ADDRSLAVE_7BIT,
                          len,
                          LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_READ);

    for (uint8_t i = 0; i < len; i++)
    {
        if (!i2c_wait_flag(LL_I2C_IsActiveFlag_RXNE, I2C1, 1))
            return false;
        data[i] = LL_I2C_ReceiveData8(I2C1);
    }

    if (!i2c_wait_flag(LL_I2C_IsActiveFlag_STOP, I2C1, 1))
        return false;

    LL_I2C_ClearFlag_STOP(I2C1);
    return true;
}


