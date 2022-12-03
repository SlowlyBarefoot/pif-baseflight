/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"

#include "drv_ak8975.h"
#include "drv_i2c.h"


// This sensor is available on MPU-9150. The accel/gyro in that chip use the same driver as MPU-6050.

// AK8975, mag sensor address
#define AK8975_MAG_I2C_ADDRESS     0x0C
#define AK8975_MAG_ID_ADDRESS      0x00
#define AK8975_MAG_DATA_ADDRESS    0x03
#define AK8975_MAG_CONTROL_ADDRESS 0x0A

static PifImuSensorAlign magAlign = IMUS_ALIGN_CW180_DEG;

static const char* hw_names = "AK8975";

static BOOL ak8975Read(int16_t *magData);

bool ak8975detect(sensorSet_t *p_sensor_set, void* p_param)
{
    bool ack = false;
    uint8_t sig = 0;

    (void)p_param;

    // device ID is in register 0 and is equal to 'H'
    ack = i2cRead(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_ID_ADDRESS, 1, &sig);
    if (!ack || sig != 'H')
        return false;

    p_sensor_set->mag.hardware = hw_names;
    p_sensor_set->mag.init = ak8975Init;
    p_sensor_set->mag.read = ak8975Read;
    return true;
}

BOOL ak8975Init(PifImuSensorAlign align)
{
    if (align > 0)
        magAlign = align;

    i2cWrite(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_CONTROL_ADDRESS, 0x01); // start reading
    return TRUE;
}

static BOOL ak8975Read(int16_t *magData)
{
    uint8_t buf[6];
    int16_t mag[3];

    i2cRead(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_DATA_ADDRESS, 6, buf);
    // align sensors to match MPU6050:
    // x -> y
    // y -> x
    // z-> -z
    mag[X] = -(int16_t)(buf[3] << 8 | buf[2]) * 4;
    mag[Y] = -(int16_t)(buf[1] << 8 | buf[0]) * 4;
    mag[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * 4;

    alignSensors(mag, magData, magAlign);

    i2cWrite(AK8975_MAG_I2C_ADDRESS, AK8975_MAG_CONTROL_ADDRESS, 0x01); // start reading again
    return TRUE;
}
