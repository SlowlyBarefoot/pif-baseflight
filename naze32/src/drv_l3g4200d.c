#include "board.h"
#include "link_driver.h"

#include "drv_i2c.h"


// L3G4200D, Standard address 0x68
#define L3G4200D_ADDRESS         0x68
#define L3G4200D_ID              0xD3
#define L3G4200D_AUTOINCR        0x80

// Registers
#define L3G4200D_WHO_AM_I        0x0F
#define L3G4200D_CTRL_REG1       0x20
#define L3G4200D_CTRL_REG2       0x21
#define L3G4200D_CTRL_REG3       0x22
#define L3G4200D_CTRL_REG4       0x23
#define L3G4200D_CTRL_REG5       0x24
#define L3G4200D_REFERENCE       0x25
#define L3G4200D_STATUS_REG      0x27
#define L3G4200D_GYRO_OUT        0x28

// Bits
#define L3G4200D_POWER_ON        0x0F
#define L3G4200D_FS_SEL_2000DPS  0xF0
#define L3G4200D_DLPF_32HZ       0x00
#define L3G4200D_DLPF_54HZ       0x40
#define L3G4200D_DLPF_78HZ       0x80
#define L3G4200D_DLPF_93HZ       0xC0

static uint8_t mpuLowPassFilter = L3G4200D_DLPF_32HZ;
static PifImuSensorAlign gyroAlign = IMUS_ALIGN_CW0_DEG;

static const char* hw_names = "L3G4200D";

static BOOL l3g4200dInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL l3g4200dRead(sensorSet_t *p_sensor_set, float *gyro_data);

bool l3g4200dDetect(sensorSet_t *p_sensor_set, void* p_param)
{
    uint8_t deviceid;

    (void)p_param;

    pif_Delay1ms(25);

    i2cRead(L3G4200D_ADDRESS, L3G4200D_WHO_AM_I, 1, &deviceid);
    if (deviceid != L3G4200D_ID)
        return false;

    p_sensor_set->gyro.init = l3g4200dInit;
    p_sensor_set->gyro.read = l3g4200dRead;

    // 14.2857dps/lsb scalefactor
    p_sensor_set->gyro.scale = (((32767.0f / 14.2857f) * M_PI) / ((32767.0f / 4.0f) * 180.0f * 1000000.0f));

    // default LPF is set to 32Hz
    switch (p_sensor_set->gyro.lpf) {
        default:
        case 32:
            mpuLowPassFilter = L3G4200D_DLPF_32HZ;
            break;
        case 54:
            mpuLowPassFilter = L3G4200D_DLPF_54HZ;
            break;
        case 78:
            mpuLowPassFilter = L3G4200D_DLPF_78HZ;
            break;
        case 93:
            mpuLowPassFilter = L3G4200D_DLPF_93HZ;
            break;
    }

    p_sensor_set->gyro.hardware = hw_names;
    return true;
}

static BOOL l3g4200dInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    bool ack;

    (void)p_sensor_set;

    pif_Delay1ms(100);

    ack = i2cWrite(L3G4200D_ADDRESS, L3G4200D_CTRL_REG4, L3G4200D_FS_SEL_2000DPS);
    if (!ack)
        failureMode(3);

    pif_Delay1ms(5);
    i2cWrite(L3G4200D_ADDRESS, L3G4200D_CTRL_REG1, L3G4200D_POWER_ON | mpuLowPassFilter);
    if (align > 0)
        gyroAlign = align;
    return TRUE;
}

// Read 3 gyro values into user-provided buffer. No overrun checking is done.
static BOOL l3g4200dRead(sensorSet_t *p_sensor_set, float *gyro_data)
{
    uint8_t buf[6];
    int16_t data[3];

    (void)p_sensor_set;

    i2cRead(L3G4200D_ADDRESS, L3G4200D_AUTOINCR | L3G4200D_GYRO_OUT, 6, buf);
    data[X] = (int16_t)((buf[0] << 8) | buf[1]) / 4;
    data[Y] = (int16_t)((buf[2] << 8) | buf[3]) / 4;
    data[Z] = (int16_t)((buf[4] << 8) | buf[5]) / 4;

    alignSensors(data, gyro_data, gyroAlign);
    return TRUE;
}
