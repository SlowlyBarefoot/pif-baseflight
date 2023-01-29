/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"

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

static BOOL hmc5883lRead(int16_t *magData);

bool hmc5883lDetect(sensorSet_t *p_sensor_set, void* p_param)
{
    (void)p_param;

    if (!pifHmc5883_Init(&hmc5883, PIF_ID_AUTO, &g_i2c_port, &imu_sensor)) return false;

    p_sensor_set->mag.hardware = hw_names;
    p_sensor_set->mag.init = hmc5883lInit;
    p_sensor_set->mag.read = hmc5883lRead;
    return true;
}

BOOL hmc5883lInit(PifImuSensorAlign align)
{
    gpio_config_t gpio;
    int16_t adc[3];
    int i;
    int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.
    bool bret = true;           // Error indicator
    PifHmc5883ConfigA config_a;

    pifImuSensor_SetMagAlign(&imu_sensor, align);

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
    config_a.byte = 0;
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_POS_BIAS;
    config_a.bit.data_rate = HMC5883_DATARATE_15HZ;
    pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte);   // Reg A DOR = 0x010 + MS1, MS0 set to pos bias
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.
    pifHmc5883_SetGain(&hmc5883, HMC5883_GAIN_2_5GA); // Set the Gain to 2.5Ga (7:5->011)
    pif_Delay1ms(100);
    hmc5883lRead(adc);

    for (i = 0; i < 10; i++) {  // Collect 10 samples
        pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_SINGLE);
        pif_Delay1ms(50);
        hmc5883lRead(adc);       // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged rather than taking the max.
        xyz_total[X] += adc[X];
        xyz_total[Y] += adc[Y];
        xyz_total[Z] += adc[Z];

        // Detect saturation.
        if (-4096 >= min(adc[X], min(adc[Y], adc[Z]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        actLed1Toggle();
    }

    // Apply the negative bias. (Same gain)
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_NEG_BIAS;
    pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte);   // Reg A DOR = 0x010 + MS1, MS0 set to negative bias.
    for (i = 0; i < 10; i++) {
        pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_SINGLE);
        pif_Delay1ms(50);
        hmc5883lRead(adc);               // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged.
        xyz_total[X] -= adc[X];
        xyz_total[Y] -= adc[Y];
        xyz_total[Z] -= adc[Z];

        // Detect saturation.
        if (-4096 >= min(adc[X], min(adc[Y], adc[Z]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        actLed1Toggle();
    }

    hmc5883.scale[X] = fabsf(660.0f * HMC58X3_X_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[X]);
    hmc5883.scale[Y] = fabsf(660.0f * HMC58X3_Y_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Y]);
    hmc5883.scale[Z] = fabsf(660.0f * HMC58X3_Z_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Z]);

    // leave test mode
    config_a.bit.measure_mode = HMC5883_MEASURE_MODE_NORMAL;
    config_a.bit.samples = HMC5883_SAMPLES_8;
    pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_CONFIG_A, config_a.byte);         // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    pifHmc5883_SetGain(&hmc5883, HMC5883_GAIN_1_3GA);                                       // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    pifI2cDevice_WriteRegByte(hmc5883._p_i2c, HMC5883_REG_MODE, HMC5883_MODE_CONTINOUS);    // Mode register             -- 000000 00    continuous Conversion Mode
    pif_Delay1ms(100);

    if (!bret) {                // Something went wrong so get a best guess
        hmc5883.scale[X] = 1.0f;
        hmc5883.scale[Y] = 1.0f;
        hmc5883.scale[Z] = 1.0f;
    }
    return TRUE;
}

static BOOL hmc5883lRead(int16_t *magData)
{
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
    return pifImuSensor_ReadMag2(&imu_sensor, magData);
}
