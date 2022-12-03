/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

#include "core/pif_i2c.h"

//#define USE_I2C_POLLING
//#define USE_I2C_INTERRUPT
#define USE_I2C_DMA

PifI2cReturn actI2cWrite(uint8_t addr_, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size);
PifI2cReturn actI2cRead(uint8_t addr_, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size);
