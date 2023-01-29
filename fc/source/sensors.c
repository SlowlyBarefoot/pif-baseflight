/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "buzzer.h"
#include "mw.h"

#ifndef __PIF_NO_LOG__
	#include "core/pif_log.h"
#endif
#include "filter/pif_noise_filter_int32.h"

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
uint16_t calibratingG = 0;
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.
int16_t heading, magHold;

#ifdef BARO
static int32_t baroPressure = 0;
static int32_t baroTemperature = 0;
#endif

extern uint16_t InflightcalibratingA;
extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;
extern uint16_t batteryWarningVoltage;
extern uint16_t batteryCriticalVoltage;
extern uint8_t batteryCellCount;

sensorSet_t sensor_set = {
	.gyro.hardware = NULL,
	.acc.hardware = NULL,		// acc access functions
	.mag.hardware = NULL, 		// mag access functions
	.mag.declination = 0.0f,
	.baro.hardware = NULL		// barometer access functions
};
PifImuSensor imu_sensor;

bool sensorsAutodetect(sensorDetect_t* gyroDetect, sensorDetect_t* accDetect, sensorDetect_t* baroDetect, sensorDetect_t* magDetect)
{
    int16_t deg, min;

    pifImuSensor_Init(&imu_sensor);
    pifImuSensor_InitBoardAlignment(&imu_sensor, mcfg.board_align_roll, mcfg.board_align_pitch, mcfg.board_align_yaw);

    // set gyro low pass filter parameters
    sensor_set.gyro.lpf = mcfg.gyro_lpf;

    // Autodetect Invensense gyro hardware
    while (gyroDetect->p_func) {
    	if ((*gyroDetect->p_func)(&sensor_set, gyroDetect->p_param)) {
    	    sensorsSet(SENSOR_GYRO);
    		break;
    	}
    	gyroDetect++;
    }
    if (!sensor_set.gyro.hardware) return false;

    // Autodetect Invensense acc hardware
    if (sensor_set.acc.hardware) {
    	sensorsSet(SENSOR_ACC);
    }
    else if (accDetect) {
        while (accDetect->p_func) {
        	if ((*accDetect->p_func)(&sensor_set, accDetect->p_param)) {
            	sensorsSet(SENSOR_ACC);
        		break;
        	}
        	accDetect++;
        }
    }

#ifdef BARO
    // Autodetect Invensense baro hardware
    if (sensor_set.baro.hardware) {
    	sensorsSet(SENSOR_BARO);
    }
    else if (baroDetect) {
		while (baroDetect->p_func) {
			if ((*baroDetect->p_func)(&sensor_set, baroDetect->p_param)) {
				sensorsSet(SENSOR_BARO);
				break;
			}
			baroDetect++;
		}
    }
#else
    (void)baroDetect;
#endif

    // Now time to init things, acc first
    if (sensors(SENSOR_ACC))
        sensor_set.acc.init(mcfg.acc_align);
    // this is safe because either mpu6050 or mpu3050 or lg3d20 sets it, and in case of fail, we never get here.
    sensor_set.gyro.init(mcfg.gyro_align);

#ifdef MAG
    // Autodetect Invensense mag hardware
    if (sensor_set.mag.hardware) {
    	sensorsSet(SENSOR_MAG);
    }
    else if (magDetect) {
		while (magDetect->p_func) {
			if ((*magDetect->p_func)(&sensor_set, magDetect->p_param)) {
				sensorsSet(SENSOR_MAG);
				break;
			}
			magDetect++;
		}
    }
#else
    (void)magDetect;
#endif

    // calculate magnetic declination
    deg = cfg.mag_declination / 100;
    min = cfg.mag_declination % 100;
    if (sensors(SENSOR_MAG))
        sensor_set.mag.declination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
    else
        sensor_set.mag.declination = 0.0f;

    return true;
}

uint16_t RSSI_getValue(void)
{
    uint16_t value = 0;

    if (mcfg.rssi_aux_channel > 0) {
        const int16_t rssiChannelData = rcData[AUX1 + mcfg.rssi_aux_channel - 1];
        // Range of rssiChannelData is [1000;2000]. rssi should be in [0;1023];
        value = (uint16_t)((constrain(rssiChannelData - 1000, 0, mcfg.rssi_aux_max) / (float) mcfg.rssi_aux_max) * 1023.0f);
    } else if (mcfg.rssi_adc_channel > 0) {
        const int16_t rssiData = (((int32_t)(actGetAdcChannel(ADC_RSSI) - mcfg.rssi_adc_offset)) * 1023L) / mcfg.rssi_adc_max;
        // Set to correct range [0;1023]
        value = constrain(rssiData, 0, 1023);
    }

    // return range [0;1023]
    return value;
}

void batteryInit(void)
{
    uint32_t i;
    float voltage = 0;

    // average up some voltage readings
    for (i = 0; i < 32; i++) {
        voltage += actGetBatteryVoltage() * mcfg.vbatscale;
        pif_Delay1ms(10);
    }

    voltage = voltage / 32;
#ifndef __PIF_NO_LOG__
    pifLog_Printf(LT_INFO, "Battery: %fV", voltage / 10);
#endif

    // autodetect cell count, going from 2S..8S
    for (i = 1; i < 8; i++) {
        if (voltage < i * mcfg.vbatmaxcellvoltage)
            break;
    }
    batteryCellCount = i;
    batteryWarningVoltage = i * mcfg.vbatwarningcellvoltage; // 3.5V per cell minimum, configurable in CLI
    batteryCriticalVoltage = i * mcfg.vbatmincellvoltage; // 3.3V per cell minimum, configurable in CLI
}

static void ACC_Common(void)
{
    static int32_t a[3];
    int axis;

    if (calibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == CALIBRATING_ACC_CYCLES)
                a[axis] = 0;
            // Sum up CALIBRATING_ACC_CYCLES readings
            a[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            mcfg.accZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (calibratingA == 1) {
            mcfg.accZero[ROLL] = (a[ROLL] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
            mcfg.accZero[PITCH] = (a[PITCH] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
            mcfg.accZero[YAW] = (a[YAW] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc_1G;
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeEEPROM(1, true);      // write accZero in EEPROM
        }
        calibratingA--;
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        static int32_t b[3];
        static int16_t accZero_saved[3] = { 0, 0, 0 };
        static int16_t angleTrim_saved[2] = { 0, 0 };
        // Saving old zeropoints before measurement
        if (InflightcalibratingA == 50) {
            accZero_saved[ROLL] = mcfg.accZero[ROLL];
            accZero_saved[PITCH] = mcfg.accZero[PITCH];
            accZero_saved[YAW] = mcfg.accZero[YAW];
            angleTrim_saved[ROLL] = cfg.angleTrim[ROLL];
            angleTrim_saved[PITCH] = cfg.angleTrim[PITCH];
        }
        if (InflightcalibratingA > 0) {
            for (axis = 0; axis < 3; axis++) {
                // Reset a[axis] at start of calibration
                if (InflightcalibratingA == 50)
                    b[axis] = 0;
                // Sum up 50 readings
                b[axis] += accADC[axis];
                // Clear global variables for next reading
                accADC[axis] = 0;
                mcfg.accZero[axis] = 0;
            }
            // all values are measured
            if (InflightcalibratingA == 1) {
                AccInflightCalibrationActive = false;
                AccInflightCalibrationMeasurementDone = true;
                buzzer(BUZZER_ACC_CALIBRATION);      // buzzer for indicatiing the end of calibration
                // recover saved values to maintain current flight behavior until new values are transferred
                mcfg.accZero[ROLL] = accZero_saved[ROLL];
                mcfg.accZero[PITCH] = accZero_saved[PITCH];
                mcfg.accZero[YAW] = accZero_saved[YAW];
                cfg.angleTrim[ROLL] = angleTrim_saved[ROLL];
                cfg.angleTrim[PITCH] = angleTrim_saved[PITCH];
            }
            InflightcalibratingA--;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (AccInflightCalibrationSavetoEEProm) {      // the copter is landed, disarmed and the combo has been done again
            AccInflightCalibrationSavetoEEProm = false;
            mcfg.accZero[ROLL] = b[ROLL] / 50;
            mcfg.accZero[PITCH] = b[PITCH] / 50;
            mcfg.accZero[YAW] = b[YAW] / 50 - acc_1G;    // for nunchuk 200=1G
            cfg.angleTrim[ROLL] = 0;
            cfg.angleTrim[PITCH] = 0;
            writeEEPROM(1, true);          // write accZero in EEPROM
        }
    }

    accADC[ROLL] -= mcfg.accZero[ROLL];
    accADC[PITCH] -= mcfg.accZero[PITCH];
    accADC[YAW] -= mcfg.accZero[YAW];
}

BOOL ACC_getADC(void)
{
    if (sensor_set.acc.read(accADC)) {
    	ACC_Common();
    	return TRUE;
    }
	return FALSE;
}

#ifdef BARO
static void Baro_Common(void)
{
    static int32_t baroHistTab[BARO_TAB_SIZE_MAX];
    static int baroHistIdx = 0;
    int indexplus1;

    indexplus1 = (baroHistIdx + 1);
    if (indexplus1 >= cfg.baro_tab_size)
        indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;
}

void evtBaroRead(float pressure, float temperature)
{
    baroPressure = pressure;
    baroTemperature = temperature * 100;
    Baro_Common();
    pifTask_SetTrigger(sensor_set.baro.p_b_task);
}
#endif /* BARO */

typedef struct stdev_t {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

static void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

static void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

static float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

static float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}

static void GYRO_Common(void)
{
    int axis;
    static int32_t g[3];
    static stdev_t var[3];

    if (calibratingG > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == CALIBRATING_GYRO_CYCLES) {
                g[axis] = 0;
                devClear(&var[axis]);
            }
            // Sum up 1000 readings
            g[axis] += gyroADC[axis];
            devPush(&var[axis], (float)gyroADC[axis]);
            // Clear global variables for next reading
            gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                float dev = devStandardDeviation(&var[axis]);
                // check deviation and startover if idiot was moving the model
                if (mcfg.moron_threshold && dev > mcfg.moron_threshold) {
#ifndef __PIF_NO_LOG__
                    pifLog_Printf(LT_INFO, "Gyro Calib[%u]: %f > %d", axis, dev, mcfg.moron_threshold);
#endif
                    calibratingG = CALIBRATING_GYRO_CYCLES;
                    devClear(&var[0]);
                    devClear(&var[1]);
                    devClear(&var[2]);
                    g[0] = g[1] = g[2] = 0;
                    continue;
                }
                gyroZero[axis] = (g[axis] + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
                blinkLED(10, 15, 1);
            }
        }
        calibratingG--;
#ifndef __PIF_NO_LOG__
        if (!calibratingG) pifLog_Printf(LT_INFO, "Gyro Zero: %d %d %d", gyroZero[X], gyroZero[Y], gyroZero[Z]);
#endif
    }
    for (axis = 0; axis < 3; axis++)
        gyroADC[axis] -= gyroZero[axis];
}

BOOL Gyro_getADC(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    if (sensor_set.gyro.read(gyroADC)) {
    	GYRO_Common();
    	return TRUE;
    }
    return FALSE;
}

#ifdef MAG
static uint8_t magInit = 0;

void Mag_init(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    actLed1State(ON);
    if (!sensor_set.mag.init(mcfg.mag_align)) {
#ifndef __PIF_NO_LOG__
    	pifLog_Printf(LT_INFO, "Mag Error:%d", pif_error);
#endif
    }
    actLed1State(OFF);
    magInit = 1;
}

uint16_t taskMagGetAdc(PifTask *p_task)
{
    static uint32_t tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint32_t axis;

    (void)p_task;

    // Read mag sensor
    if (!sensor_set.mag.read(magADC)) {
    	pifTask_SetTrigger(p_task);
    	return 0;
    }

    if (f.CALIBRATE_MAG) {
        tCal = (*pif_act_timer1us)();
        for (axis = 0; axis < 3; axis++) {
            mcfg.magZero[axis] = 0;
            magZeroTempMin[axis] = magADC[axis];
            magZeroTempMax[axis] = magADC[axis];
        }
        f.CALIBRATE_MAG = 0;
    }

    if (magInit) {              // we apply offset only once mag calibration is done
        magADC[X] -= mcfg.magZero[X];
        magADC[Y] -= mcfg.magZero[Y];
        magADC[Z] -= mcfg.magZero[Z];
    }

    if (tCal != 0) {
        if (((*pif_act_timer1us)() - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            actLed0Toggle();
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin[axis])
                    magZeroTempMin[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax[axis])
                    magZeroTempMax[axis] = magADC[axis];
            }
        } else {
            tCal = 0;
            for (axis = 0; axis < 3; axis++)
                mcfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2; // Calculate offsets
            writeEEPROM(1, true);
        }
    }
    return 0;
}
#endif

#ifdef SONAR

static PifNoiseFilter* p_sonar_noise_filter;

static float getSonarDistance(int32_t distance)
{
	if (distance > 300)
		distance = -1;
	sonarDistance = distance;

#ifdef BARO
	return baroTemperature / 100.0;
#else
	return 20.0;
#endif
}

static float getSonarDistanceFilter(int32_t distance)
{
	distance = *(int32_t*)pifNoiseFilter_Process(p_sonar_noise_filter, &distance);
	if (distance > 300)
		distance = -1;
	sonarDistance = distance;

#ifdef BARO
	return baroTemperature / 100.0;
#else
	return 20.0;
#endif
}

void Sonar_init(sonarInitFuncPtr sonarInit, sonar_filter_t filter)
{
    static PifNoiseFilterInt32 noise_filter;

    // the repeat interval of trig signal should be greater than 60ms
    // to avoid interference between connective measurements.
	switch (filter) {
	case SF_AVERAGE:
	    if (pifNoiseFilterInt32_Init(&noise_filter, 5)) {
	        p_sonar_noise_filter = &noise_filter.parent;
	    }
		break;

	case SF_NOISE_CANCEL:
	    if (pifNoiseFilterInt32_Init(&noise_filter, 7)) {
	        if (pifNoiseFilterInt32_SetNoiseCancel(&noise_filter)) {
	            p_sonar_noise_filter = &noise_filter.parent;
	        }
	    }
		break;

	default:
		break;
	}
	if (p_sonar_noise_filter) {
		if (!(*sonarInit)(60, getSonarDistanceFilter)) return;
	}
	else {
		if (!(*sonarInit)(60, getSonarDistance)) return;
	}
    sensorsSet(SENSOR_SONAR);
    sonarDistance = -1;
#ifndef __PIF_NO_LOG__
	pifLog_Print(LT_INFO, "Sonar init : success");
#endif
}

#endif
