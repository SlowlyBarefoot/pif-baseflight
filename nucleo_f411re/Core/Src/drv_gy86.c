/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "link_driver.h"

#include "drv_gy86.h"

#include "core/pif_i2c.h"
#include "core/pif_log.h"
#include "sensor/pif_gy86.h"
#include "sensor/pif_imu_sensor.h"

/* Generic driver for invensense gyro/acc devices.
 *
 * Supported hardware:
 * MPU6050 (gyro + acc)
 *
 * AUX_I2C is enabled on devices which have bypass, to allow forwarding to compass in MPU9150-style devices
 */

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.


// Hardware access functions
static BOOL mpu6050Init(sensorSet_t *p_sensor_set, PifGy86Config* p_config);

// General forward declarations
static BOOL mpuAccInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL mpuAccRead(sensorSet_t *p_sensor_set, int16_t *accData);
static BOOL mpuGyroInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL mpuGyroRead(sensorSet_t *p_sensor_set, int16_t *gyroData);
static BOOL hmc5883lInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL hmc5883lRead(sensorSet_t *p_sensor_set, int16_t *magData);

// Hardware access function
static PifGy86 s_gy86;

// sync this with AccelSensors enum from board.h
static const char* hw_Names[] = { "MPU9250", "AK8963", "BMP280" };


bool gy86Detect(sensorSet_t *p_sensor_set, void* p_param)
{
	PifGy86Config config;

    (void)p_param;

    // Set acc_1G. Modified once by mpu6050CheckRevision for old (hopefully nonexistent outside of clones) parts
    p_sensor_set->acc.acc_1G = 512 * 8;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    p_sensor_set->gyro.scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;

    memset(&config, 0, sizeof(PifGy86Config));

    config.disallow_yield_id = DISALLOW_YIELD_ID_I2C;

    config.mpu60x0_clksel = MPU60X0_CLKSEL_PLL_ZGYRO;
    // default lpf is 42Hz, 255 is special case of nolpf
    if (p_sensor_set->gyro.lpf == 255)
        config.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A260HZ_G256HZ;
    else if (p_sensor_set->gyro.lpf >= 188)
        config.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A184HZ_G188HZ;
    else if (p_sensor_set->gyro.lpf >= 98)
        config.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A94HZ_G98HZ;
    else if (p_sensor_set->gyro.lpf >= 42)
        config.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A44HZ_G42HZ;
    else if (p_sensor_set->gyro.lpf >= 20)
        config.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A21HZ_G20HZ;
    else if (p_sensor_set->gyro.lpf >= 10)
        config.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A10HZ_G10HZ;
    else
        config.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A5HZ_G5HZ;
    config.mpu60x0_fs_sel = MPU60X0_FS_SEL_2000DPS;
    config.mpu60x0_afs_sel = MPU60X0_AFS_SEL_8G;
    config.mpu60x0_i2c_mst_clk = MPU60X0_I2C_MST_CLK_400KHZ;

    config.hmc5883_gain = HMC5883_GAIN_1_3GA;
    config.hmc5883_samples = HMC5883_SAMPLES_8;
    config.hmc5883_data_rate = HMC5883_DATARATE_75HZ;
    config.hmc5883_mode = HMC5883_MODE_CONTINOUS;

#ifdef BARO
    config.ms5611_osr = MS5611_OSR_4096;
    config.ms5611_read_period = 25;												// 25ms
    config.ms5611_evt_read = p_sensor_set->baro.evt_read;
#endif

    // initialize the device
    if (!mpu6050Init(p_sensor_set, &config)) return false;

    p_sensor_set->gyro.hardware = hw_Names[0];
    p_sensor_set->acc.hardware = hw_Names[0];
    p_sensor_set->mag.hardware = hw_Names[1];
    p_sensor_set->baro.hardware = hw_Names[2];

    return true;
}

static BOOL mpu6050Init(sensorSet_t *p_sensor_set, PifGy86Config* p_config)
{
    if (!pifGy86_Init(&s_gy86, PIF_ID_AUTO, &g_i2c_port, &p_sensor_set->imu_sensor, p_config)) return FALSE;
    s_gy86._mpu6050.gyro_scale = 4;
    s_gy86._mpu6050.temp_scale = 100;

    p_sensor_set->acc.init = mpuAccInit;
    p_sensor_set->acc.read = mpuAccRead;
    p_sensor_set->gyro.init = mpuGyroInit;
    p_sensor_set->gyro.read = mpuGyroRead;
    p_sensor_set->mag.init = hmc5883lInit;
    p_sensor_set->mag.read = hmc5883lRead;
    return TRUE;
}

static BOOL mpuAccInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    pifImuSensor_SetAccelAlign(&p_sensor_set->imu_sensor, align);
    return TRUE;
}

static BOOL mpuAccRead(sensorSet_t *p_sensor_set, int16_t *accData)
{
	int16_t data[3];

    if (!pifImuSensor_ReadAccel2(&p_sensor_set->imu_sensor, data)) return FALSE;
    accData[0] = data[0];
    accData[1] = data[1];
    accData[2] = data[2];
    return TRUE;
}

static BOOL mpuGyroInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    pifImuSensor_SetGyroAlign(&p_sensor_set->imu_sensor, align);
    return TRUE;
}

static BOOL mpuGyroRead(sensorSet_t *p_sensor_set, int16_t *gyroData)
{
	int16_t data[3];

	if (!pifImuSensor_ReadGyro2(&p_sensor_set->imu_sensor, data)) return FALSE;
	gyroData[0] = data[0];
	gyroData[1] = data[1];
	gyroData[2] = data[2];
	return TRUE;
}

static BOOL hmc5883lInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    int16_t magADC[3];
    int i;
    int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.
    bool bret = true;           // Error indicator
    PifHmc5883ConfigA config_a;

    pifImuSensor_SetMagAlign(&p_sensor_set->imu_sensor, align);

    if (!pifI2cDevice_WriteRegBit8(s_gy86._mpu6050._p_i2c, MPU60X0_REG_USER_CTRL, MPU60X0_USER_CTRL_I2C_MST_EN, FALSE)) return FALSE;

    if (!pifI2cDevice_WriteRegBit8(s_gy86._mpu6050._p_i2c, MPU60X0_REG_INT_PIN_CFG, MPU60X0_INT_PIN_CFG_I2C_BYPASS_EN, TRUE)) return FALSE;

    config_a.byte = 0;
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_POS_BIAS;
    config_a.bit.data_rate = HMC5883_DATARATE_15HZ;
    if (!pifI2cDevice_WriteRegByte(s_gy86._hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte)) return FALSE;   // Reg A DOR = 0x010 + MS1, MS0 set to pos bias
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.
    if (!pifHmc5883_SetGain(&s_gy86._hmc5883, HMC5883_GAIN_2_5GA)) return FALSE; // Set the Gain to 2.5Ga (7:5->011)
    pifTaskManager_YieldMs(100);
    pifHmc5883_ReadMag(&s_gy86._hmc5883, magADC);

    for (i = 0; i < 10; i++) {  // Collect 10 samples
        if (!pifI2cDevice_WriteRegByte(s_gy86._hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_SINGLE)) return FALSE;
        pifTaskManager_YieldMs(50);
        pifHmc5883_ReadMag(&s_gy86._hmc5883, magADC);       // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged rather than taking the max.
        xyz_total[X] += magADC[X];
        xyz_total[Y] += magADC[Y];
        xyz_total[Z] += magADC[Z];

        // Detect saturation.
        if (-4096 >= min(magADC[X], min(magADC[Y], magADC[Z]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        actLed1Toggle();
    }

    // Apply the negative bias. (Same gain)
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_NEG_BIAS;
    if (!pifI2cDevice_WriteRegByte(s_gy86._hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte)) return FALSE;   // Reg A DOR = 0x010 + MS1, MS0 set to negative bias.
    for (i = 0; i < 10; i++) {
        if (!pifI2cDevice_WriteRegByte(s_gy86._hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_SINGLE)) return FALSE;
        pifTaskManager_YieldMs(50);
        pifHmc5883_ReadMag(&s_gy86._hmc5883, magADC);               // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged.
        xyz_total[X] -= magADC[X];
        xyz_total[Y] -= magADC[Y];
        xyz_total[Z] -= magADC[Z];

        // Detect saturation.
        if (-4096 >= min(magADC[X], min(magADC[Y], magADC[Z]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        actLed1Toggle();
    }

    s_gy86._hmc5883.scale[X] = fabsf(660.0f * HMC58X3_X_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[X]);
    s_gy86._hmc5883.scale[Y] = fabsf(660.0f * HMC58X3_Y_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Y]);
    s_gy86._hmc5883.scale[Z] = fabsf(660.0f * HMC58X3_Z_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Z]);

    // leave test mode
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_NORMAL;
    config_a.bit.samples = HMC5883_SAMPLES_8;
    if (!pifI2cDevice_WriteRegByte(s_gy86._hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte)) return FALSE;		// Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    if (!pifHmc5883_SetGain(&s_gy86._hmc5883, HMC5883_GAIN_1_3GA)) return FALSE;                               		// Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    if (!pifI2cDevice_WriteRegByte(s_gy86._hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_CONTINOUS)) return FALSE;	// Mode register             -- 000000 00    continuous Conversion Mode

    if (!pifI2cDevice_WriteRegBit8(s_gy86._mpu6050._p_i2c, MPU60X0_REG_INT_PIN_CFG, MPU60X0_INT_PIN_CFG_I2C_BYPASS_EN, FALSE)) return FALSE;

    if (!pifI2cDevice_WriteRegBit8(s_gy86._mpu6050._p_i2c, MPU60X0_REG_USER_CTRL, MPU60X0_USER_CTRL_I2C_MST_EN, TRUE)) return FALSE;

    if (!bret) {                // Something went wrong so get a best guess
    	s_gy86._hmc5883.scale[X] = 1.0f;
    	s_gy86._hmc5883.scale[Y] = 1.0f;
    	s_gy86._hmc5883.scale[Z] = 1.0f;
    }

    s_gy86._ms5611._p_task->pause = FALSE;

    pifLog_Printf(LT_INFO, "Mag scale: %f %f %f", (double)s_gy86._hmc5883.scale[X], (double)s_gy86._hmc5883.scale[Y], (double)s_gy86._hmc5883.scale[Z]);
    return TRUE;
}

static BOOL hmc5883lRead(sensorSet_t *p_sensor_set, int16_t *magData)
{
	int16_t data[3];

	// During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
	if (!pifImuSensor_ReadMag2(&p_sensor_set->imu_sensor, data)) return FALSE;
	magData[0] = data[0];
	magData[1] = data[1];
	magData[2] = data[2];
	return TRUE;
}
