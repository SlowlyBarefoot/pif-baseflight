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

#include "stm32f10x_conf.h"
#include "core_cm3.h"

#ifndef __CC_ARM
// only need this garbage on gcc
#define USE_LAME_PRINTF
#include "printf.h"
#endif

#include "utils.h"


#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#define MAX_MOTORS  12
#define MAX_SERVOS  8
#define MAX_PWM_INPUTS  8
#define MAX_PPM_INPUTS 16

typedef enum HardwareRevision {
    NAZE32 = 1,                                         // Naze32 and compatible with 8MHz HSE
    NAZE32_REV5,                                        // Naze32 and compatible with 12MHz HSE
    NAZE32_SP,                                          // Naze32 w/Sensor Platforms
    NAZE32_REV6,                                        // Naze32 rev6
} HardwareRevision;


// current auto-detected hardware revision (enum HardwareRevision in board.h)
extern int hw_revision;

extern const char* g_board_name;

// Hardware definitions and GPIO
// Target definitions (NAZE, CJMCU, ... are same as in Makefile
#if defined(NAZE)
// Afroflight32

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3 // PB3 (LED)
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_4 // PB4 (LED)
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_12 // PA12 (Buzzer)
#define BARO_GPIO   GPIOC
#define BARO_PIN    Pin_13
#define INV_PIN     Pin_2 // PB2 (BOOT1) abused as inverter select GPIO
#define INV_GPIO    GPIOB

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
#define INVERTER
#define TELEMETRY
#define MOTOR_PWM_RATE 400

#define I2C_DEVICE (I2CDEV_2)

// #define PROD_DEBUG
// #define SOFT_I2C                 // enable to test software i2c
// #define SOFT_I2C_PB1011          // If SOFT_I2C is enabled above, need to define pinout as well (I2C1 = PB67, I2C2 = PB1011)
// #define SOFT_I2C_PB67

// AfroFlight32

#elif defined(CJMCU)
// CJMCU brushed quad pcb

#define LED0_GPIO   GPIOC
#define LED0_PIN    Pin_13 // PC13 (LED)
#define LED0
#define LED1_GPIO   GPIOC
#define LED1_PIN    Pin_14 // PC14 (LED)
#define LED1
#define LED2_GPIO   GPIOC
#define LED2_PIN    Pin_15 // PC15 (LED)
#define LED2

#define GYRO
#define ACC
#define MAG
#define MOTOR_PWM_RATE 16000

#define I2C_DEVICE (I2CDEV_1)

#else
#error TARGET NOT DEFINED!
#endif /* all conditions */

