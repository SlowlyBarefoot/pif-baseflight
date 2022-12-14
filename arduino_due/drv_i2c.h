/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

#include "core/pif_i2c.h"


//#define USE_I2C_WIRE


#ifndef USE_I2C_WIRE

#define I2C_CLOCK_100KHz	100000
#define I2C_CLOCK_400KHz	400000


#ifdef __cplusplus
extern "C" {
#endif

void I2C_Init(uint32_t clock);
void I2C_Exit();

void I2C_XmitTimeout(uint32_t timeout);
void I2C_RecvTimeout(uint32_t timeout);

BOOL I2C_ReadAddr(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint8_t size);
BOOL I2C_Read(uint8_t addr, uint8_t* p_data, uint8_t size);

BOOL I2C_WriteAddr(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint8_t size);
BOOL I2C_Write(uint8_t addr, uint8_t* p_data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif

PifI2cReturn actI2cWrite(uint8_t addr_, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size);
PifI2cReturn actI2cRead(uint8_t addr_, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size);
