/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "utils.h"


#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

#define MAX_MOTORS  12
#define MAX_SERVOS  8
#define MAX_PWM_INPUTS  8
#define MAX_PPM_INPUTS 16


extern const char g_board_name[];


// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
    ACC_BMA280 = 4,
    ACC_MPU6500 = 5,
    ACC_NONE = 6
} AccelSensors;

typedef enum CompassSensors {
    MAG_DEFAULT = 0,
    MAG_HMC5883L = 1,
    MAG_AK8975 = 2,
    MAG_NONE = 3
} CompassSensors;

typedef enum BaroSensors {
    BARO_DEFAULT = 0,
    BARO_BMP280 = 1,
    BARO_BMP085 = 2,
    BARO_MS5611 = 3,
    BARO_NONE = 4
} BaroSensors;

typedef enum {
    ALIGN_DEFAULT = 0,                                      // driver-provided alignment
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;

enum {
    GYRO_UPDATED = 1 << 0,
    ACC_UPDATED = 1 << 1,
    MAG_UPDATED = 1 << 2,
    TEMP_UPDATED = 1 << 3
};


// sonar
typedef void (*EvtSonarEcho)(bool state);


#define GYRO
#define ACC
#define MAG
#define BARO
#define GPS
//#define LEDRING
#define SONAR
#define BUZZER
#define LED0
#define LED1
//#define INVERTER
//#define TELEMETRY
#define MOTOR_PWM_RATE 400

#define SENSORS_SET (SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)

// #define SOFT_I2C                 // enable to test software i2c
// #define SOFT_I2C_PB1011          // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

