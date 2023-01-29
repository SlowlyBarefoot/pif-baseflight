/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"

#include "drv_gpio.h"
#include "drv_i2c.h"
#include "drv_mpu.h"
#include "drv_spi.h"

#include "sensor/pif_mpu30x0.h"
#include "sensor/pif_mpu60x0.h"
#include "sensor/pif_mpu6500.h"

/* Generic driver for invensense gyro/acc devices.
 *
 * Supported hardware:
 * MPU3050 (gyro only)
 * MPU6050 (gyro + acc)
 * MPU6500 (gyro + acc)
 *
 * AUX_I2C is enabled on devices which have bypass, to allow forwarding to compass in MPU9150-style devices
 */

// This is generally where all Invensense devices are at, for default (AD0 down) I2C address
#define MPU_ADDRESS                         (0x68)
#define GYRO_INT_GPIO                       (GPIOC)
#define GYRO_INT_PIN                        (Pin_13)

#define MPU_RA_WHO_AM_I                     (0x75)
// For debugging/identification purposes
#define MPU_RA_XA_OFFS_H                    (0x06)    //[15:0] XA_OFFS
#define MPU_RA_PRODUCT_ID                   (0x0C)    // Product ID Register

// WHO_AM_I register contents for MPU3050, 6050 and 6500
#define MPU6500_WHO_AM_I_CONST              (0x70)
#define MPUx0x0_WHO_AM_I_CONST              (0x68)


// Hardware access functions
static bool mpuReadRegisterI2C(uint8_t reg, uint8_t *data, int length);
static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data);
static bool mpuReadRegisterSPI(uint8_t reg, uint8_t *data, int length);
static bool mpuWriteRegisterSPI(uint8_t reg, uint8_t data);
static void mpu3050Init(sensorSet_t *p_sensor_set);
static void mpu6050Init(sensorSet_t *p_sensor_set);
static void mpu6500Init(sensorSet_t *p_sensor_set);

// Hardware access funcptrs
typedef bool (*mpuReadRegPtr)(uint8_t reg, uint8_t *data, int length);
typedef bool (*mpuWriteRegPtr)(uint8_t reg, uint8_t data);
typedef void (*mpuInitPtr)(sensorSet_t *p_sensor_set);
// General forward declarations
static void mpu6050CheckRevision(void);
#ifdef PROD_DEBUG
static void mpu6050SelfTest(void);
#endif
static BOOL dummyInit(PifImuSensorAlign align);
static BOOL dummyRead(int16_t *data);
static BOOL mpuAccInit(PifImuSensorAlign align);
static BOOL mpuAccRead(int16_t *acc_data);
static BOOL mpuGyroInit(PifImuSensorAlign align);
static BOOL mpuGyroRead(int16_t *gyro_data);

typedef struct mpu_access_t {
    mpuReadRegPtr read;
    mpuWriteRegPtr write;
    mpuInitPtr init;

    union {
        PifMpu30x0 mpu30x0;
        PifMpu60x0 mpu60x0;
        PifMpu6500 mpu6500;
    };
} mpu_access_t;

// Needed for MPU6050 half-scale acc bug
extern uint16_t acc_1G;
// Hardware access function
static mpu_access_t mpu;

static const char* hw_names[] = { "MPU3050", "MPU6050", "MPU6500" };


bool mpuDetect(sensorSet_t *p_sensor_set, void* p_param)
{
    bool ack, useSpi = false;
    uint8_t sig = 0, legacy = 0;
    mpu_hardware_e hw = MPU_NONE;
    gpio_config_t gpio;

    (void)p_param;

    // Set acc_1G. Modified once by mpu6050CheckRevision for old (hopefully nonexistent outside of clones) parts
    acc_1G = 512 * 8;

    // Try I2C access first
    mpu.read = mpuReadRegisterI2C;
    mpu.write = mpuWriteRegisterI2C;

    ack = mpu.read(MPU_RA_WHO_AM_I, &sig, 1);

    if (!ack) {
        // Let's see if there's something on SPI
        mpu.read = mpuReadRegisterSPI;
        mpu.write = mpuWriteRegisterSPI;
        useSpi = true;

        mpu.read(MPU_RA_WHO_AM_I, &sig, 1);
    } else {
        // Special case for MPU30x0, WHO_AM_I is in a different place
        mpu.read(MPU30X0_REG_WHO_AM_I, &legacy, 1);
        legacy &= 0x7E;
        if (legacy == MPUx0x0_WHO_AM_I_CONST) {
            hw = MPU_3050;
            mpu.init = mpu3050Init;
        }
    }

    sig &= 0x7E; // mask the lower/upper bits per MPUxxxx spec

    if (sig == MPUx0x0_WHO_AM_I_CONST) {
        hw = MPU_60x0;
        mpu6050CheckRevision();
#ifdef PROD_DEBUG
        mpu6050SelfTest();
#endif
        mpu.init = mpu6050Init;
    } else if (sig == MPU6500_WHO_AM_I_CONST) {
        if (useSpi)
            hw = MPU_65xx_SPI;
        else
            hw = MPU_65xx_I2C;
        mpu.init = mpu6500Init;
    }

    // We're done. Nothing was found on any bus.
    if (hw == MPU_NONE)
        return false;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    p_sensor_set->gyro.scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;

    // MPU_INT output on rev5+ hardware (PC13). rev4 was on PB13, conflicts with SPI devices
    if (hw_revision >= NAZE32_REV5) {
        gpio.pin = GYRO_INT_PIN;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(GYRO_INT_GPIO, &gpio);
    }

    // initialize the device
    mpu.init(p_sensor_set);

    // MPU6500 on I2C bus
    if (g_crystal_clock == 12000000 && hw == MPU_65xx_I2C)
        hw_revision = NAZE32_REV6;

    return true;
}

static void mpu3050Init(sensorSet_t *p_sensor_set)
{
    PifMpu30x0DlpfFsSync dlpf_fs_sync;
    PifMpu30x0UserCtrl user_ctrl;
    PifMpu30x0PwrMgmt pwr_mgmt;

    pifMpu30x0_Init(&mpu.mpu30x0, PIF_ID_AUTO, &g_i2c_port, MPU30X0_I2C_ADDR, &imu_sensor);

    pifI2cDevice_WriteRegByte(mpu.mpu30x0._p_i2c, MPU30X0_REG_SMPLRT_DIV, 0);

    dlpf_fs_sync.byte = 0;
    dlpf_fs_sync.bit.fs_sel = MPU30X0_FS_SEL_2000DPS;
    // default lpf is 42Hz, 255 is special case of nolpf
    if (p_sensor_set->gyro.lpf == 255)
        dlpf_fs_sync.bit.dlpf_cfg = MPU30X0_DLPF_CFG_256HZ;
    else if (p_sensor_set->gyro.lpf >= 188)
        dlpf_fs_sync.bit.dlpf_cfg = MPU30X0_DLPF_CFG_188HZ;
    else if (p_sensor_set->gyro.lpf >= 98)
        dlpf_fs_sync.bit.dlpf_cfg = MPU30X0_DLPF_CFG_98HZ;
    else if (p_sensor_set->gyro.lpf >= 42)
        dlpf_fs_sync.bit.dlpf_cfg = MPU30X0_DLPF_CFG_42HZ;
    else if (p_sensor_set->gyro.lpf >= 20)
        dlpf_fs_sync.bit.dlpf_cfg = MPU30X0_DLPF_CFG_20HZ;
    else if (p_sensor_set->gyro.lpf >= 10)
        dlpf_fs_sync.bit.dlpf_cfg = MPU30X0_DLPF_CFG_10HZ;
    else
        dlpf_fs_sync.bit.dlpf_cfg = MPU30X0_DLPF_CFG_5HZ;
    pifMpu30x0_SetDlpfFsSync(&mpu.mpu30x0, dlpf_fs_sync);

    pifI2cDevice_WriteRegByte(mpu.mpu30x0._p_i2c, MPU30X0_REG_INT_CFG, 0);

    user_ctrl.byte = 0;
    user_ctrl.bit.gyro_rst = 0;
    pifI2cDevice_WriteRegByte(mpu.mpu30x0._p_i2c, MPU30X0_REG_USER_CTRL, user_ctrl.byte);

    pwr_mgmt.byte = 0;
    pwr_mgmt.bit.clk_sel = MPU30X0_CLK_SEL_PLL_XGYRO;
    pifI2cDevice_WriteRegByte(mpu.mpu30x0._p_i2c, MPU30X0_REG_PWR_MGMT, pwr_mgmt.byte);

    p_sensor_set->acc.init = dummyInit;
    p_sensor_set->acc.read = dummyRead;
    p_sensor_set->gyro.hardware = hw_names[0];
    p_sensor_set->gyro.init = mpuGyroInit;
    p_sensor_set->gyro.read = mpuGyroRead;
}

static void mpu6050Init(sensorSet_t *p_sensor_set)
{
    PifMpu60x0PwrMgmt1 pwr_mgmt_1;
    PifMpu60x0Config config;
    PifMpu60x0GyroConfig gyro_config;
    PifMpu60x0AccelConfig accel_config;

    pifMpu60x0_Init(&mpu.mpu60x0, PIF_ID_AUTO, &g_i2c_port, MPU60X0_I2C_ADDR(0), &imu_sensor);

    pifI2cDevice_WriteRegByte(mpu.mpu60x0._p_i2c, MPU60X0_REG_SMPLRT_DIV, 0x00); // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)

    pwr_mgmt_1.byte = 0;
    pwr_mgmt_1.bit.clksel = MPU60X0_CLKSEL_PLL_ZGYRO;
    pifI2cDevice_WriteRegByte(mpu.mpu60x0._p_i2c, MPU60X0_REG_PWR_MGMT_1, pwr_mgmt_1.byte); // Clock source = 3 (PLL with Z Gyro reference)
    pif_Delay1ms(10);

    config.byte = 0;
    // default lpf is 42Hz, 255 is special case of nolpf
    if (p_sensor_set->gyro.lpf == 255)
        config.bit.dlpf_cfg = MPU60X0_DLPF_CFG_A260HZ_G256HZ;
    else if (p_sensor_set->gyro.lpf >= 188)
        config.bit.dlpf_cfg = MPU60X0_DLPF_CFG_A184HZ_G188HZ;
    else if (p_sensor_set->gyro.lpf >= 98)
        config.bit.dlpf_cfg = MPU60X0_DLPF_CFG_A94HZ_G98HZ;
    else if (p_sensor_set->gyro.lpf >= 42)
        config.bit.dlpf_cfg = MPU60X0_DLPF_CFG_A44HZ_G42HZ;
    else if (p_sensor_set->gyro.lpf >= 20)
        config.bit.dlpf_cfg = MPU60X0_DLPF_CFG_A21HZ_G20HZ;
    else if (p_sensor_set->gyro.lpf >= 10)
        config.bit.dlpf_cfg = MPU60X0_DLPF_CFG_A10HZ_G10HZ;
    else
        config.bit.dlpf_cfg = MPU60X0_DLPF_CFG_A5HZ_G5HZ;
    pifI2cDevice_WriteRegByte(mpu.mpu60x0._p_i2c, MPU60X0_REG_CONFIG, config.byte);

    // Gyro config
    gyro_config.byte = 0;
    gyro_config.bit.fs_sel = MPU60X0_FS_SEL_2000DPS;
    pifMpu60x0_SetGyroConfig(&mpu.mpu60x0, gyro_config); // full-scale 2kdps gyro range

    // Accel scale 8g (4096 LSB/g)
    accel_config.byte = 0;
    accel_config.bit.afs_sel = MPU60X0_AFS_SEL_8G;
    pifMpu60x0_SetAccelConfig(&mpu.mpu60x0, accel_config);

    // Data ready interrupt configuration
    pifI2cDevice_WriteRegByte(mpu.mpu60x0._p_i2c, MPU60X0_REG_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_RD_CLEAR_DIS, I2C_BYPASS_EN

    pifI2cDevice_WriteRegByte(mpu.mpu60x0._p_i2c, MPU60X0_REG_INT_ENABLE, 0x01); // DATA_RDY_EN interrupt enable

    p_sensor_set->acc.hardware = hw_names[1];
    p_sensor_set->acc.init = mpuAccInit;
    p_sensor_set->acc.read = mpuAccRead;
    p_sensor_set->gyro.hardware = hw_names[1];
    p_sensor_set->gyro.init = mpuGyroInit;
    p_sensor_set->gyro.read = mpuGyroRead;
}

static void mpu6500Init(sensorSet_t *p_sensor_set)
{
    PifMpu6500PwrMgmt1 pwr_mgmt_1;
    PifMpu6500GyroConfig gyro_config;
    PifMpu6500Config config;
    PifMpu6500AccelConfig accel_config;

    pifMpu6500_Init(&mpu.mpu6500, PIF_ID_AUTO, &g_i2c_port, MPU6500_I2C_ADDR(0), &imu_sensor);

    // Device reset
    pifI2cDevice_WriteRegByte(mpu.mpu6500._p_i2c, MPU6500_REG_PWR_MGMT_1, 0x80); // Device reset
    pif_Delay1ms(100);

    pifI2cDevice_WriteRegByte(mpu.mpu6500._p_i2c, MPU6500_REG_SIGNAL_PATH_RESET, 0x07); // Signal path reset
    pif_Delay1ms(100);
    // Gyro config

    pwr_mgmt_1.byte = 0;
    pwr_mgmt_1.bit.clksel = 1; // Clock source = 1 (Auto-select PLL or else intrc)
    pifI2cDevice_WriteRegByte(mpu.mpu6500._p_i2c, MPU6500_REG_PWR_MGMT_1, pwr_mgmt_1.byte);
    pif_Delay1ms(10);

    gyro_config.byte = 0;
    gyro_config.bit.gyro_fs_sel = MPU6500_GYRO_FS_SEL_2000DPS;
    pifMpu6500_SetGyroConfig(&mpu.mpu6500, gyro_config);

    config.byte = 0;
    // default lpf is 42Hz, 255 is special case of nolpf
    if (p_sensor_set->gyro.lpf >= 250)
        config.bit.dlpf_cfg = MPU6500_DLPF_CFG_250HZ;
    else if (p_sensor_set->gyro.lpf >= 184)
        config.bit.dlpf_cfg = MPU6500_DLPF_CFG_184HZ;
    else if (p_sensor_set->gyro.lpf >= 92)
        config.bit.dlpf_cfg = MPU6500_DLPF_CFG_92HZ;
    else if (p_sensor_set->gyro.lpf >= 41)
        config.bit.dlpf_cfg = MPU6500_DLPF_CFG_41HZ;
    else if (p_sensor_set->gyro.lpf >= 20)
        config.bit.dlpf_cfg = MPU6500_DLPF_CFG_20HZ;
    else if (p_sensor_set->gyro.lpf >= 10)
        config.bit.dlpf_cfg = MPU6500_DLPF_CFG_10HZ;
    else
        config.bit.dlpf_cfg = MPU6500_DLPF_CFG_5HZ;
    pifI2cDevice_WriteRegByte(mpu.mpu6500._p_i2c, MPU6500_REG_CONFIG, config.byte);

    pifI2cDevice_WriteRegByte(mpu.mpu6500._p_i2c, MPU6500_REG_SMPLRT_DIV, 0); // 1kHz S/R

    // Accel config
    accel_config.byte = 0;
    accel_config.bit.accel_fs_sel = MPU6500_ACCEL_FS_SEL_8G;
    pifMpu6500_SetAccelConfig(&mpu.mpu6500, accel_config);

    // Data ready interrupt configuration
    pifI2cDevice_WriteRegByte(mpu.mpu6500._p_i2c, MPU6500_REG_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN

    pifI2cDevice_WriteRegByte(mpu.mpu6500._p_i2c, MPU6500_REG_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable

    mpu.mpu6500.gyro_scale = 4;

    p_sensor_set->acc.hardware = hw_names[2];
    p_sensor_set->acc.init = mpuAccInit;
    p_sensor_set->acc.read = mpuAccRead;
    p_sensor_set->gyro.hardware = hw_names[2];
    p_sensor_set->gyro.init = mpuGyroInit;
    p_sensor_set->gyro.read = mpuGyroRead;
}

static BOOL dummyInit(PifImuSensorAlign align)
{
    (void)align;
    return TRUE;
}

static BOOL dummyRead(int16_t *data)
{
    (void)data;
    return TRUE;
}

static BOOL mpuAccInit(PifImuSensorAlign align)
{
    pifImuSensor_SetAccelAlign(&imu_sensor, align);
    return TRUE;
}

static BOOL mpuGyroInit(PifImuSensorAlign align)
{
    pifImuSensor_SetGyroAlign(&imu_sensor, align);
    return TRUE;
}

static BOOL mpuAccRead(int16_t *acc_data)
{
    return pifImuSensor_ReadAccel2(&imu_sensor, acc_data);
}

static BOOL mpuGyroRead(int16_t *gyro_data)
{
    return pifImuSensor_ReadGyro2(&imu_sensor, gyro_data);
}

static void mpu6050CheckRevision(void)
{
    uint8_t rev;
    uint8_t tmp[6];
    int half = 0;

    // determine product ID and accel revision
    mpu.read(MPU_RA_XA_OFFS_H, tmp, 6);
    rev = ((tmp[5] & 0x01) << 2) | ((tmp[3] & 0x01) << 1) | (tmp[1] & 0x01);
    if (rev) {
        // Congrats, these parts are better
        if (rev == 1) {
            half = 1;
        } else if (rev == 2) {
            half = 0;
        } else {
            failureMode(5);
        }
    } else {
        mpu.read(MPU_RA_PRODUCT_ID, &rev, 1);
        rev &= 0x0F;
        if (!rev) {
            failureMode(5);
        } else if (rev == 4) {
            half = 1;
        } else {
            half = 0;
        }
    }

    // All this just to set the value
    if (half)
        acc_1G = 255 * 8;
}

#ifdef PROD_DEBUG

// MPU6xxx registers
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_FIFO_COUNT_H     0x72
#define MPU_RA_FIFO_R_W         0x74

// MPU6050 bits
#define MPU6050_BIT_FIFO_RST    0x04
#define MPU6050_BIT_DMP_RST     0x08
#define MPU6050_BIT_FIFO_EN     0x40

#define INV_X_GYRO              0x40
#define INV_Y_GYRO              0x20
#define INV_Z_GYRO              0x10
#define INV_XYZ_GYRO            (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL           0x08

// Self-test constants
#define GYRO_SENS           (32768 / 250)
#define ACCEL_SENS          (32768 / 16)
#define MAX_ACCEL_VAR       (0.14f)
#define MIN_G               (0.3f)
#define MAX_G               (0.95f)
#define MIN_DPS             (10.f)
#define MAX_DPS             (105.f)
#define MAX_GYRO_VAR        (0.14f)

#define MPU6050_MAX_PACKET_LEN  12

static void mpu6050GetBiases(int32_t *gyro, int32_t *accel, int hw_test)
{
    uint8_t data[MPU6050_MAX_PACKET_LEN];
    int fifo_count, packet_count, i;

    // get self-test biases
    mpu.write(MPU_RA_PWR_MGMT_1, 0x01);
    mpu.write(MPU_RA_PWR_MGMT_2, 0x00);
    pif_Delay1ms(200);
    mpu.write(MPU_RA_INT_ENABLE, 0);
    mpu.write(MPU_RA_FIFO_EN, 0);
    mpu.write(MPU_RA_PWR_MGMT_1, 0);
    mpu.write(MPU_RA_I2C_MST_CTRL, 0);
    mpu.write(MPU_RA_USER_CTRL, 0);
    mpu.write(MPU_RA_USER_CTRL, MPU6050_BIT_FIFO_RST | MPU6050_BIT_DMP_RST);
    pif_Delay1ms(15);
    mpu.write(MPU_RA_CONFIG, 1); // 188Hz
    mpu.write(MPU_RA_SMPLRT_DIV, 0); // 1kHz
    if (hw_test) {
        mpu.write(MPU_RA_GYRO_CONFIG, 0 | 0xE0); // 250dps w/test
        mpu.write(MPU_RA_ACCEL_CONFIG, 0x18 | 0xE0); // 16g w/test
        pif_Delay1ms(200);
    } else {
        mpu.write(MPU_RA_GYRO_CONFIG, 0); // 250dps w/o test
        mpu.write(MPU_RA_ACCEL_CONFIG, 0x18); // 16g w/o test
    }

    // Enable FIFO
    mpu.write(MPU_RA_USER_CTRL, MPU6050_BIT_FIFO_EN);
    mpu.write(MPU_RA_FIFO_EN, INV_XYZ_GYRO | INV_XYZ_ACCEL);

    // Read back test samples
    mpu.read(MPU_RA_FIFO_COUNT_H, data, 2);
    fifo_count = (data[0] << 8) | data[1];
    packet_count = fifo_count / MPU6050_MAX_PACKET_LEN;
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    for (i = 0; i < packet_count; i++) {
        int16_t accel_cur[3], gyro_cur[3];
        mpu.read(MPU_RA_FIFO_R_W, data, MPU6050_MAX_PACKET_LEN);
        accel_cur[0] = ((int16_t)data[0] << 8) | data[1];
        accel_cur[1] = ((int16_t)data[2] << 8) | data[3];
        accel_cur[2] = ((int16_t)data[4] << 8) | data[5];
        accel[0] += (int32_t)accel_cur[0];
        accel[1] += (int32_t)accel_cur[1];
        accel[2] += (int32_t)accel_cur[2];
        gyro_cur[0] = (((int16_t)data[6] << 8) | data[7]);
        gyro_cur[1] = (((int16_t)data[8] << 8) | data[9]);
        gyro_cur[2] = (((int16_t)data[10] << 8) | data[11]);
        gyro[0] += (int32_t)gyro_cur[0];
        gyro[1] += (int32_t)gyro_cur[1];
        gyro[2] += (int32_t)gyro_cur[2];
    }

    gyro[0] = (int32_t)(((int64_t)gyro[0] << 16) / GYRO_SENS / packet_count);
    gyro[1] = (int32_t)(((int64_t)gyro[1] << 16) / GYRO_SENS / packet_count);
    gyro[2] = (int32_t)(((int64_t)gyro[2] << 16) / GYRO_SENS / packet_count);
    accel[0] = (int32_t)(((int64_t)accel[0] << 16) / ACCEL_SENS / packet_count);
    accel[1] = (int32_t)(((int64_t)accel[1] << 16) / ACCEL_SENS / packet_count);
    accel[2] = (int32_t)(((int64_t)accel[2] << 16) / ACCEL_SENS / packet_count);
    // Don't remove gravity!
    if (accel[2] > 0L)
        accel[2] -= 65536L;
    else
        accel[2] += 65536L;
}

static int mpu6050AccelSelfTest(int32_t *bias_regular, int32_t *bias_st)
{
    uint8_t tmp[4], shift_code[3];
    float st_shift[3], st_shift_cust, st_shift_var;
    const float accel_max_z_bias = (.23f * 65535.f), accel_max_xy_bias = (.18f * 65535.f);
    int i, result = 0;

    // Get accel production shift
    mpu.read(0x0D, tmp, 4);
    shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
    shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
    shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
    for (i = 0; i < 3; i++) {
        if (!shift_code[i]) {
            st_shift[i] = 0.f;
            continue;
        }
        /* Equivalent to..
         * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
         */
        st_shift[i] = 0.34f;
        while (--shift_code[i])
            st_shift[i] *= 1.034f;
    }

    for (i = 0; i < 3; i++) {
        st_shift_cust = abs(bias_regular[i] - bias_st[i]) / 65536.f;
        if (st_shift[i]) {
            st_shift_var = st_shift_cust / st_shift[i] - 1.f;
            if (fabs(st_shift_var) > MAX_ACCEL_VAR)
                result |= 1 << i;
        } else if ((st_shift_cust < MIN_G) || (st_shift_cust > MAX_G)) {
            result |= 1 << i;
        }
    }

    if (result == 0) {
        if (bias_regular[0] > accel_max_xy_bias)
            result |= 1;
        if (bias_regular[1] > accel_max_xy_bias)
            result |= 2;
        if (bias_regular[2] > accel_max_z_bias)
            result |= 4;
    }

    return result;
}

static int mpu6050GyroSelfTest(int32_t *bias_regular, int32_t *bias_st)
{
    int i, result = 0;
    uint8_t tmp[3];
    float st_shift, st_shift_cust, st_shift_var;
    const float gyro_max_bias = (20.f * 65535.f);

    mpu.read(0x0D, tmp, 3);

    tmp[0] &= 0x1F;
    tmp[1] &= 0x1F;
    tmp[2] &= 0x1F;

    for (i = 0; i < 3; i++) {
        st_shift_cust = abs(bias_regular[i] - bias_st[i]) / 65536.f;
        if (tmp[i]) {
            st_shift = 3275.f / GYRO_SENS;
            while (--tmp[i])
                st_shift *= 1.046f;
            st_shift_var = st_shift_cust / st_shift - 1.f;
            if (fabs(st_shift_var) > MAX_GYRO_VAR)
                result |= 1 << i;
        } else if ((st_shift_cust < MIN_DPS) || (st_shift_cust > MAX_DPS))
            result |= 1 << i;
    }

    if (result == 0) {
        if (bias_regular[0] > gyro_max_bias)
            result |= 1;
        if (bias_regular[1] > gyro_max_bias)
            result |= 2;
        if (bias_regular[2] > gyro_max_bias)
            result |= 4;
    }

    return result;
}

static void mpu6050SelfTest(void)
{
    int32_t accel[3], gyro[3];
    int32_t gyro_st[3], accel_st[3];
    int gresult = 0, aresult = 0;

    mpu6050GetBiases(gyro, accel, 0);
    mpu6050GetBiases(gyro, accel, 1);
    aresult = mpu6050AccelSelfTest(accel, accel_st);
    gresult = mpu6050GyroSelfTest(gyro, gyro_st);

    if (!aresult || !gresult)
        failureMode(10);
}
#endif

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t *data, int length)
{
    return i2cRead(MPU_ADDRESS, reg, length, data);
}

static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
    return i2cWrite(MPU_ADDRESS, reg, data);
}

static bool mpuReadRegisterSPI(uint8_t reg, uint8_t *data, int length)
{
#ifndef CJMCU
    spiSelect(true);
    spiTransferByte(reg | 0x80); // read transaction
    spiTransfer(data, NULL, length);
    spiSelect(false);
#endif
    return true;
}

static bool mpuWriteRegisterSPI(uint8_t reg, uint8_t data)
{
#ifndef CJMCU
    spiSelect(true);
    spiTransferByte(reg);
    spiTransferByte(data);
    spiSelect(false);
#endif
    return true;
}
