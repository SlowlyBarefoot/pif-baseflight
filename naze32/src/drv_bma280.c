#include "board.h"
#include "mw.h"

#include "drv_bma280.h"
#include "drv_i2c.h"


// BMA280, default I2C address mode 0x18
#define BMA280_ADDRESS     0x18
#define BMA280_ACC_X_LSB   0x02
#define BMA280_PMU_BW      0x10
#define BMA280_PMU_RANGE   0x0F

extern uint16_t acc_1G;

static BOOL bma280Init(PifImuSensorAlign align);
static BOOL bma280Read(int16_t *accelData);

static PifImuSensorAlign accAlign = IMUS_ALIGN_CW0_DEG;

static const char* hw_names = "BMA280";

bool bma280Detect(sensorSet_t *p_sensor_set, void* p_param)
{
    bool ack = false;
    uint8_t sig = 0;

    (void)p_param;

    ack = i2cRead(BMA280_ADDRESS, 0x00, 1, &sig);
    if (!ack || sig != 0xFB)
        return false;

    p_sensor_set->acc.hardware = hw_names;
    p_sensor_set->acc.init = bma280Init;
    p_sensor_set->acc.read = bma280Read;
    return true;
}

static BOOL bma280Init(PifImuSensorAlign align)
{
    i2cWrite(BMA280_ADDRESS, BMA280_PMU_RANGE, 0x08); // +-8g range
    i2cWrite(BMA280_ADDRESS, BMA280_PMU_BW, 0x0E); // 500Hz BW

    acc_1G = 512 * 8;

    if (align > 0)
        accAlign = align;
    return TRUE;
}

static BOOL bma280Read(int16_t *accelData)
{
    uint8_t buf[6];
    int16_t data[3];

    i2cRead(BMA280_ADDRESS, BMA280_ACC_X_LSB, 6, buf);

    // Data format is lsb<5:0><crap><new_data_bit> | msb<13:6>
    data[0] = (int16_t)((buf[0] >> 2) + (buf[1] << 8));
    data[1] = (int16_t)((buf[2] >> 2) + (buf[3] << 8));
    data[2] = (int16_t)((buf[4] >> 2) + (buf[5] << 8));

    alignSensors(data, accelData, accAlign);
    return TRUE;
}
