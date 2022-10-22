/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

#include "baseflight.h"
#include "utils.h"

#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>


#define MAX_MOTORS  12
#define MAX_SERVOS  8
#define MAX_PWM_INPUTS  8
#define MAX_PPM_INPUTS 16


extern const char g_board_name[];


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
//#define TELEMETRY

#define MOTOR_PWM_RATE 400
