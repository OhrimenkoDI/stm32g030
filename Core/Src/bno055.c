#include "bno055.h"
#include "i2c_ll.h"
#include "stm32g0xx_ll_utils.h"

static void delay_ms(uint32_t ms)
{
    LL_mDelay(ms);
}

/* ===== Low-level helpers ===== */

static bool write_reg(uint8_t reg, uint8_t val)
{
    return i2c_ll_write(BNO055_ADDR, reg, &val, 1);
}

static bool read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2c_ll_read(BNO055_ADDR, reg, buf, len);
}

/* ===== Init ===== */

bool bno055_init(void)
{
    uint8_t id = 0;

    if (!read_regs(BNO055_CHIP_ID_ADDR, &id, 1))
        return false;

    if (id != BNO055_ID)
        return false;

    /* CONFIG */
    write_reg(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    delay_ms(25);

    /* Reset */
    write_reg(BNO055_SYS_TRIGGER_ADDR, 0x20);
    delay_ms(650);

    /* CONFIG again */
    write_reg(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    delay_ms(25);

    /* External crystal */
    write_reg(BNO055_SYS_TRIGGER_ADDR, 0x80);
    delay_ms(10);

    /* Units */
    write_reg(BNO055_UNIT_SEL_ADDR, 0x00);

    /* Axis map (как в ESP) */
    write_reg(BNO055_AXIS_MAP_CONFIG_ADDR, 0x24);
    write_reg(BNO055_AXIS_MAP_SIGN_ADDR, 0x00);

    /* NDOF */
    write_reg(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    delay_ms(100);

    return true;
}

/* ===== Calibration ===== */

bool bno055_read_calibration(uint8_t *sys, uint8_t *gyro,
                             uint8_t *accel, uint8_t *mag)
{
    uint8_t c;
    if (!read_regs(BNO055_CALIB_STAT_ADDR, &c, 1))
        return false;

    *mag   = (c >> 0) & 0x03;
    *accel = (c >> 2) & 0x03;
    *gyro  = (c >> 4) & 0x03;
    *sys   = (c >> 6) & 0x03;

    return true;
}

/* ===== Sensors ===== */

bool bno055_read_euler(float *heading, float *roll, float *pitch)
{
    uint8_t d[6];
    if (!read_regs(BNO055_EULER_H_LSB_ADDR, d, 6))
        return false;

    int16_t h = (d[1] << 8) | d[0];
    int16_t r = (d[3] << 8) | d[2];
    int16_t p = (d[5] << 8) | d[4];

    *heading = h / 16.0f;
    *roll    = r / 16.0f;
    *pitch   = p / 16.0f;

    return true;
}

bool bno055_read_quaternion(float *qw, float *qx, float *qy, float *qz)
{
    uint8_t d[8];
    if (!read_regs(BNO055_QUATERNION_DATA_W_LSB, d, 8))
        return false;

    *qw = ((int16_t)(d[1] << 8 | d[0])) / 16384.0f;
    *qx = ((int16_t)(d[3] << 8 | d[2])) / 16384.0f;
    *qy = ((int16_t)(d[5] << 8 | d[4])) / 16384.0f;
    *qz = ((int16_t)(d[7] << 8 | d[6])) / 16384.0f;

    return true;
}

bool bno055_read_accel(float *ax, float *ay, float *az)
{
    uint8_t d[6];
    if (!read_regs(BNO055_ACCEL_DATA_X_LSB_ADDR, d, 6))
        return false;

    *ax = ((int16_t)(d[1] << 8 | d[0])) / 100.0f;
    *ay = ((int16_t)(d[3] << 8 | d[2])) / 100.0f;
    *az = ((int16_t)(d[5] << 8 | d[4])) / 100.0f;

    return true;
}

bool bno055_read_gyro(float *gx, float *gy, float *gz)
{
    uint8_t d[6];
    if (!read_regs(BNO055_GYRO_DATA_X_LSB_ADDR, d, 6))
        return false;

    *gx = ((int16_t)(d[1] << 8 | d[0])) / 16.0f;
    *gy = ((int16_t)(d[3] << 8 | d[2])) / 16.0f;
    *gz = ((int16_t)(d[5] << 8 | d[4])) / 16.0f;

    return true;
}

bool bno055_read_mag(float *mx, float *my, float *mz)
{
    uint8_t d[6];
    if (!read_regs(BNO055_MAG_DATA_X_LSB_ADDR, d, 6))
        return false;

    *mx = ((int16_t)(d[1] << 8 | d[0])) / 16.0f;
    *my = ((int16_t)(d[3] << 8 | d[2])) / 16.0f;
    *mz = ((int16_t)(d[5] << 8 | d[4])) / 16.0f;

    return true;
}

bool bno055_read_linear_accel(float *ax, float *ay, float *az)
{
    uint8_t d[6];
    if (!read_regs(BNO055_LINEAR_ACCEL_DATA_X_LSB, d, 6))
        return false;

    *ax = ((int16_t)(d[1] << 8 | d[0])) / 100.0f;
    *ay = ((int16_t)(d[3] << 8 | d[2])) / 100.0f;
    *az = ((int16_t)(d[5] << 8 | d[4])) / 100.0f;

    return true;
}

bool bno055_read_gravity(float *gx, float *gy, float *gz)
{
    uint8_t d[6];
    if (!read_regs(BNO055_GRAVITY_DATA_X_LSB, d, 6))
        return false;

    *gx = ((int16_t)(d[1] << 8 | d[0])) / 100.0f;
    *gy = ((int16_t)(d[3] << 8 | d[2])) / 100.0f;
    *gz = ((int16_t)(d[5] << 8 | d[4])) / 100.0f;

    return true;
}
