/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "drv_i2c.h"

#ifndef __PIF_NO_LOG__
	#include "core/pif_log.h"
#endif

#include <Wire.h>


PifI2cReturn actI2cWrite(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	uint8_t error;
	int i;
	uint16_t n;

	Wire.beginTransmission(addr);
	if (isize > 0) {
		for (i = isize - 1; i >= 0; i--) {
			Wire.write((iaddr >> (i * 8)) & 0xFF);
		}
	}
    for (n = 0; n < size; n++) {
    	Wire.write(p_data[n]);
    }
    error = Wire.endTransmission();
    if (error != 0) goto fail;
    return IR_COMPLETE;

fail:
#ifndef __PIF_NO_LOG__
	pifLog_Printf(LT_ERROR, "I2CW(%Xh) IA=%lXh IS=%u S=%u E=%u", addr, iaddr, isize, size, error);
#endif
	return IR_ERROR;
}

PifI2cReturn actI2cRead(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	uint8_t error;
	int i;
	uint8_t count;
	uint16_t n;

	if (isize > 0) {
		Wire.beginTransmission(addr);
		for (i = isize - 1; i >= 0; i--) {
			Wire.write((iaddr >> (i * 8)) & 0xFF);
		}
	    error = Wire.endTransmission();
	    if (error != 0) goto fail;
	}

    count = Wire.requestFrom(addr, (uint8_t)size);
    if (count < size) goto fail;

    for (n = 0; n < size; n++) {
    	p_data[n] = Wire.read();
    }
    return IR_COMPLETE;

fail:
#ifndef __PIF_NO_LOG__
	pifLog_Printf(LT_ERROR, "I2CR(%Xh) IA=%lXh IS=%u S=%u C=%u E=%u", addr, iaddr, isize, size, count, error);
#endif
	return IR_ERROR;
}
