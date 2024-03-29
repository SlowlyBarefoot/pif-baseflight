/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "link_driver.h"

#include "drv_gpio.h"
#include "drv_i2c.h"
#include "drv_mma845x.h"


// MMA8452QT, Standard address 0x1C
// ACC_INT2 routed to PA5

#define MMA8452_ADDRESS     0x1C

#define MMA8452_DEVICE_SIGNATURE    0x2A
#define MMA8451_DEVICE_SIGNATURE    0x1A

#define MMA8452_STATUS              0x00
#define MMA8452_OUT_X_MSB           0x01
#define MMA8452_WHO_AM_I            0x0D
#define MMA8452_XYZ_DATA_CFG        0x0E
#define MMA8452_HP_FILTER_CUTOFF    0x0F
#define MMA8452_CTRL_REG1           0x2A
#define MMA8452_CTRL_REG2           0x2B
#define MMA8452_CTRL_REG3           0x2C
#define MMA8452_CTRL_REG4           0x2D
#define MMA8452_CTRL_REG5           0x2E

#define MMA8452_FS_RANGE_8G         0x02
#define MMA8452_FS_RANGE_4G         0x01
#define MMA8452_FS_RANGE_2G         0x00

#define MMA8452_HPF_CUTOFF_LV1      0x00
#define MMA8452_HPF_CUTOFF_LV2      0x01
#define MMA8452_HPF_CUTOFF_LV3      0x02
#define MMA8452_HPF_CUTOFF_LV4      0x03

#define MMA8452_CTRL_REG2_B7_ST     0x80
#define MMA8452_CTRL_REG2_B6_RST    0x40
#define MMA8452_CTRL_REG2_B4_SMODS1 0x10
#define MMA8452_CTRL_REG2_B3_SMODS0 0x08
#define MMA8452_CTRL_REG2_B2_SLPE   0x04
#define MMA8452_CTRL_REG2_B1_MODS1  0x02
#define MMA8452_CTRL_REG2_B0_MODS0  0x01

#define MMA8452_CTRL_REG2_MODS_LP   0x03
#define MMA8452_CTRL_REG2_MODS_HR   0x02
#define MMA8452_CTRL_REG2_MODS_LNLP 0x01
#define MMA8452_CTRL_REG2_MODS_NOR  0x00

#define MMA8452_CTRL_REG3_IPOL          0x02
#define MMA8452_CTRL_REG4_INT_EN_DRDY   0x01

#define MMA8452_CTRL_REG1_LNOISE        0x04
#define MMA8452_CTRL_REG1_ACTIVE        0x01

static PifImuSensorAlign accAlign = IMUS_ALIGN_CW90_DEG;

static const char* hw_names = "MMA8452";

static BOOL mma8452Init(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL mma8452Read(sensorSet_t *p_sensor_set, float *accelData);

bool mma8452Detect(sensorSet_t *p_sensor_set, void* p_param)
{
    bool ack = false;
    uint8_t sig = 0;

    (void)p_param;

    // Not supported with this frequency
    if (hw_revision >= NAZE32_REV5)
        return false;

    ack = i2cRead(MMA8452_ADDRESS, MMA8452_WHO_AM_I, 1, &sig);
    if (!ack || (sig != MMA8452_DEVICE_SIGNATURE && sig != MMA8451_DEVICE_SIGNATURE))
        return false;

    p_sensor_set->acc.hardware = hw_names;
    p_sensor_set->acc.init = mma8452Init;
    p_sensor_set->acc.read = mma8452Read;
    return true;
}

static BOOL mma8452Init(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    gpio_config_t gpio;

    (void)p_sensor_set;

    // PA5 - ACC_INT2 output on rev3/4 hardware
    gpio.pin = Pin_5;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOA, &gpio);

    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG1, 0); // Put device in standby to configure stuff
    i2cWrite(MMA8452_ADDRESS, MMA8452_XYZ_DATA_CFG, MMA8452_FS_RANGE_8G);
    i2cWrite(MMA8452_ADDRESS, MMA8452_HP_FILTER_CUTOFF, MMA8452_HPF_CUTOFF_LV4);
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG2, MMA8452_CTRL_REG2_MODS_HR | MMA8452_CTRL_REG2_MODS_HR << 3); // High resolution measurement in both sleep and active modes
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG3, MMA8452_CTRL_REG3_IPOL); // Interrupt polarity (active HIGH)
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG4, MMA8452_CTRL_REG4_INT_EN_DRDY); // Enable DRDY interrupt (unused by this driver)
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG5, 0); // DRDY routed to INT2
    i2cWrite(MMA8452_ADDRESS, MMA8452_CTRL_REG1, MMA8452_CTRL_REG1_LNOISE | MMA8452_CTRL_REG1_ACTIVE); // Turn on measurements, low noise at max scale mode, Data Rate 800Hz. LNoise mode makes range +-4G.

    p_sensor_set->acc.acc_1G = 256;

    if (align > 0)
        accAlign = align;
    return TRUE;
}

static BOOL mma8452Read(sensorSet_t *p_sensor_set, float *accelData)
{
    uint8_t buf[6];
    int16_t data[3];

    (void)p_sensor_set;

    i2cRead(MMA8452_ADDRESS, MMA8452_OUT_X_MSB, 6, buf);
    data[0] = ((int16_t)((buf[0] << 8) | buf[1]) >> 2) / 4;
    data[1] = ((int16_t)((buf[2] << 8) | buf[3]) >> 2) / 4;
    data[2] = ((int16_t)((buf[4] << 8) | buf[5]) >> 2) / 4;

    alignSensors(data, accelData, accAlign);
    return TRUE;
}
