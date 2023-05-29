/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "link_driver.h"

#include "drv_gpio.h"
#include "drv_hmc5883l.h"

#include "sensor/pif_hmc5883.h"

// HMC5883L, default address 0x1E
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

/* CTRL_REGA: Control Register A
 * Read Write
 * Default value: 0x10
 * 7:5  0   These bits must be cleared for correct operation.
 * 4:2 DO2-DO0: Data Output Rate Bits
 *             DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *            ------------------------------------------------------
 *              0  |  0   |  0   |            0.75
 *              0  |  0   |  1   |            1.5
 *              0  |  1   |  0   |            3
 *              0  |  1   |  1   |            7.5
 *              1  |  0   |  0   |           15 (default)
 *              1  |  0   |  1   |           30
 *              1  |  1   |  0   |           75
 *              1  |  1   |  1   |           Not Used
 * 1:0 MS1-MS0: Measurement Configuration Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Normal
 *              0  |  1   |  Positive Bias
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Not Used
 *
 * CTRL_REGB: Control RegisterB
 * Read Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits.
 *             GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
 *                 |      |      |  Range[Ga]    | [LSB/mGa]  |
 *            ------------------------------------------------------
 *              0  |  0   |  0   |  0.88Ga      |   1370     | 0xF800?0x07FF (-2048:2047)
 *              0  |  0   |  1   |  1.3Ga (def) |   1090     | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  0   |  1.9Ga       |   820      | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  1   |  2.5Ga       |   660      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  0   |  4.0Ga       |   440      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  1   |  4.7Ga       |   390      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  0   |  5.6Ga       |   330      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  1   |  8.1Ga       |   230      | 0xF800?0x07FF (-2048:2047)
 *                               |Not recommended|
 *
 * 4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
 *
 * _MODE_REG: Mode Register
 * Read Write
 * Default value: 0x02
 * 7:2  0   These bits must be cleared for correct operation.
 * 1:0 MD1-MD0: Mode Select Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Continuous-Conversion Mode.
 *              0  |  1   |  Single-Conversion Mode
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Sleep Mode
 */

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.

static PifHmc5883 hmc5883;

static const char* hw_names = "HMC5883L";

static BOOL hmc5883lInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL hmc5883lRead(sensorSet_t *p_sensor_set, float *magData);

bool hmc5883lDetect(sensorSet_t *p_sensor_set, void* p_param)
{
    (void)p_param;

    if (!pifHmc5883_Detect(&g_i2c_port)) return false;

    p_sensor_set->mag.hardware = hw_names;
    p_sensor_set->mag.init = hmc5883lInit;
    p_sensor_set->mag.read = hmc5883lRead;
    return true;
}

static BOOL hmc5883lInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    gpio_config_t gpio;
    PifHmc5883Param param;

    pifImuSensor_SetMagAlign(&p_sensor_set->imu_sensor, align);

    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;

    if (hw_revision == NAZE32) {
        // PB12 - MAG_DRDY output on rev4 hardware
        gpio.pin = Pin_12;
        gpioInit(GPIOB, &gpio);
    } else if (hw_revision >= NAZE32_REV5) {
        // PC14 - MAG_DRDY output on rev5 hardware
        gpio.pin = Pin_14;
        gpioInit(GPIOC, &gpio);
    }
    pif_Delay1ms(50);

	param.data_rate = HMC5883_DATARATE_DEFAULT;
	param.gain = HMC5883_GAIN_1_3GA;
	param.mode = HMC5883_MODE_CONTINOUS;
	param.samples = HMC5883_SAMPLES_8;
    if (!pifHmc5883_Init(&hmc5883, PIF_ID_AUTO, &g_i2c_port, &param, &p_sensor_set->imu_sensor)) return false;

    pif_Delay1ms(100);
    return TRUE;
}

static BOOL hmc5883lRead(sensorSet_t *p_sensor_set, float *magData)
{
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
    return pifImuSensor_ReadRawMag(&p_sensor_set->imu_sensor, magData);
}
