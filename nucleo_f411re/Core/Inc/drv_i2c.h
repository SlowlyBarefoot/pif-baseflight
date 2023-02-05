/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

#include "core/pif_i2c.h"

//#define USE_I2C_POLLING
//#define USE_I2C_INTERRUPT
#define USE_I2C_DMA

BOOL i2cInit();
