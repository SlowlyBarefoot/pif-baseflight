/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "link_driver.h"

#include "drv_adxl345.h"
#include "drv_i2c.h"


// ADXL345, Alternative address mode 0x53
#define ADXL345_ADDRESS     0x53

// Registers
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_INT_ENABLE  0x2E
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATA_OUT    0x32
#define ADXL345_FIFO_CTL    0x38

// BW_RATE values
#define ADXL345_RATE_50     0x09
#define ADXL345_RATE_100    0x0A
#define ADXL345_RATE_200    0x0B
#define ADXL345_RATE_400    0x0C
#define ADXL345_RATE_800    0x0D
#define ADXL345_RATE_1600   0x0E
#define ADXL345_RATE_3200   0x0F

// various register values
#define ADXL345_POWER_MEAS  0x08
#define ADXL345_FULL_RANGE  0x08
#define ADXL345_RANGE_2G    0x00
#define ADXL345_RANGE_4G    0x01
#define ADXL345_RANGE_8G    0x02
#define ADXL345_RANGE_16G   0x03
#define ADXL345_FIFO_STREAM 0x80

extern uint16_t acc_1G;

static BOOL adxl345Init(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL adxl345Read(sensorSet_t *p_sensor_set, float *accelData);

static bool useFifo = false;
static PifImuSensorAlign accAlign = IMUS_ALIGN_CW270_DEG;

static const char* hw_names = "ADXL345";

bool adxl345Detect(sensorSet_t *p_sensor_set, void* p_param)
{
    bool ack = false;
    uint8_t sig = 0;
    drv_adxl345_config_t* init = (drv_adxl345_config_t*)p_param;

    // Not supported with this frequency
    if (hw_revision >= NAZE32_REV5)
        return false;

    ack = i2cRead(ADXL345_ADDRESS, 0x00, 1, &sig);
    if (!ack || sig != 0xE5)
        return false;

    // use ADXL345's fifo to filter data or not
    useFifo = init->useFifo;

    p_sensor_set->acc.hardware = hw_names;
    p_sensor_set->acc.init = adxl345Init;
    p_sensor_set->acc.read = adxl345Read;
    return true;
}

static BOOL adxl345Init(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    if (useFifo) {
        uint8_t fifoDepth = 16;
        i2cWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, ADXL345_POWER_MEAS);
        i2cWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, ADXL345_FULL_RANGE | ADXL345_RANGE_8G);
        i2cWrite(ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL345_RATE_400);
        i2cWrite(ADXL345_ADDRESS, ADXL345_FIFO_CTL, (fifoDepth & 0x1F) | ADXL345_FIFO_STREAM);
    } else {
        i2cWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, ADXL345_POWER_MEAS);
        i2cWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, ADXL345_FULL_RANGE | ADXL345_RANGE_8G);
        i2cWrite(ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL345_RATE_100);
    }
    p_sensor_set->acc.acc_1G = 265; // 3.3V operation

    if (align > 0)
        accAlign = align;
    return TRUE;
}

uint8_t acc_samples = 0;

static BOOL adxl345Read(sensorSet_t *p_sensor_set, float *accelData)
{
    uint8_t buf[8];
    int16_t data[3];

    (void)p_sensor_set;

    if (useFifo) {
        int32_t x = 0;
        int32_t y = 0;
        int32_t z = 0;
        uint8_t i = 0;
        uint8_t samples_remaining;

        do {
            i++;
            i2cRead(ADXL345_ADDRESS, ADXL345_DATA_OUT, 8, buf);
            x += (int16_t)(buf[0] + (buf[1] << 8));
            y += (int16_t)(buf[2] + (buf[3] << 8));
            z += (int16_t)(buf[4] + (buf[5] << 8));
            samples_remaining = buf[7] & 0x7F;
        } while ((i < 32) && (samples_remaining > 0));
        data[0] = x / i;
        data[1] = y / i;
        data[2] = z / i;
        acc_samples = i;
    } else {
        i2cRead(ADXL345_ADDRESS, ADXL345_DATA_OUT, 6, buf);
        data[0] = buf[0] + (buf[1] << 8);
        data[1] = buf[2] + (buf[3] << 8);
        data[2] = buf[4] + (buf[5] << 8);
    }

    alignSensors(data, accelData, accAlign);
    return TRUE;
}
