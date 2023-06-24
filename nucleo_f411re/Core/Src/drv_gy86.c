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


// Hardware access functions
static BOOL mpu6050Init(sensorSet_t *p_sensor_set, PifGy86Param* p_param);

// General forward declarations
static BOOL mpuAccInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL mpuAccRead(sensorSet_t *p_sensor_set, float *accData);
static BOOL mpuGyroInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL mpuGyroRead(sensorSet_t *p_sensor_set, float *gyroData);
static BOOL hmc5883lInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL hmc5883lRead(sensorSet_t *p_sensor_set, float *magData);

// Hardware access function
static PifGy86 s_gy86;

// sync this with AccelSensors enum from board.h
static const char* hw_Names[] = { "MPU6500", "HMC5883L", "MS5611" };


bool gy86Detect(sensorSet_t *p_sensor_set, void* p_param)
{
	PifGy86Param param;

    (void)p_param;

    if (!pifGy86_Detect(&g_i2c_port)) {
        pifLog_Printf(LT_ERROR, "GY-86: Not found");
        return false;
    }

    // Set acc_1G. Modified once by mpu6050CheckRevision for old (hopefully nonexistent outside of clones) parts
    p_sensor_set->acc.acc_1G = 512 * 8;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    p_sensor_set->gyro.scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;

    memset(&param, 0, sizeof(PifGy86Param));

    param.mpu60x0_clksel = MPU60X0_CLKSEL_PLL_ZGYRO;
    // default lpf is 42Hz, 255 is special case of nolpf
    if (p_sensor_set->gyro.lpf == 255)
    	param.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A260HZ_G256HZ;
    else if (p_sensor_set->gyro.lpf >= 188)
    	param.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A184HZ_G188HZ;
    else if (p_sensor_set->gyro.lpf >= 98)
    	param.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A94HZ_G98HZ;
    else if (p_sensor_set->gyro.lpf >= 42)
    	param.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A44HZ_G42HZ;
    else if (p_sensor_set->gyro.lpf >= 20)
    	param.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A21HZ_G20HZ;
    else if (p_sensor_set->gyro.lpf >= 10)
    	param.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A10HZ_G10HZ;
    else
    	param.mpu60x0_dlpf_cfg = MPU60X0_DLPF_CFG_A5HZ_G5HZ;
    param.mpu60x0_fs_sel = MPU60X0_FS_SEL_2000DPS;
    param.mpu60x0_afs_sel = MPU60X0_AFS_SEL_8G;
    param.mpu60x0_i2c_mst_clk = MPU60X0_I2C_MST_CLK_400KHZ;

    param.hmc5883_gain = HMC5883_GAIN_1_3GA;
    param.hmc5883_samples = HMC5883_SAMPLES_8;
    param.hmc5883_data_rate = HMC5883_DATARATE_75HZ;
    param.hmc5883_mode = HMC5883_MODE_CONTINOUS;

#ifdef BARO
    param.disallow_yield_id = DISALLOW_YIELD_ID_I2C;
    param.ms5611_osr = MS5611_OSR_4096;
    param.ms5611_read_period = 25;												// 25ms
    param.ms5611_evt_read = p_sensor_set->baro.evt_read;
#endif

    // initialize the device
    if (!mpu6050Init(p_sensor_set, &param)) return false;

    p_sensor_set->gyro.hardware = hw_Names[0];
    p_sensor_set->acc.hardware = hw_Names[0];
    p_sensor_set->mag.hardware = hw_Names[1];
    p_sensor_set->baro.hardware = hw_Names[2];

    return true;
}

static BOOL mpu6050Init(sensorSet_t *p_sensor_set, PifGy86Param* p_param)
{
    if (!pifGy86_Init(&s_gy86, PIF_ID_AUTO, &g_i2c_port, p_param, &p_sensor_set->imu_sensor)) return FALSE;
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

static BOOL mpuAccRead(sensorSet_t *p_sensor_set, float *accData)
{
    return pifImuSensor_ReadRawAccel(&p_sensor_set->imu_sensor, accData);
}

static BOOL mpuGyroInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    pifImuSensor_SetGyroAlign(&p_sensor_set->imu_sensor, align);
    return TRUE;
}

static BOOL mpuGyroRead(sensorSet_t *p_sensor_set, float *gyroData)
{
	return pifImuSensor_ReadRawGyro(&p_sensor_set->imu_sensor, gyroData);
}

static BOOL hmc5883lInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    pifImuSensor_SetMagAlign(&p_sensor_set->imu_sensor, align);

    s_gy86._ms5611._p_task->pause = FALSE;
    return TRUE;
}

static BOOL hmc5883lRead(sensorSet_t *p_sensor_set, float *magData)
{
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
	return pifImuSensor_ReadRawMag(&p_sensor_set->imu_sensor, magData);
}
