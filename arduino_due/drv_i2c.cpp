/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "drv_i2c.h"

#ifndef __PIF_NO_LOG__
	#include "core/pif_log.h"
#endif

#ifdef USE_I2C_WIRE
	#include <Wire.h>
#else
	// Include Atmel CMSIS driver
	#include <include/twi.h>

	#include "variant.h"
#endif


#ifndef USE_I2C_WIRE

static Twi *twi;
static uint32_t xmit_timeout = 100000;
static uint32_t recv_timeout = 100000;


static BOOL TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout)
{
	uint32_t _status_reg = 0;

	while ((_status_reg & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK) {
			pif_error = E_RECEIVE_NACK;
			return FALSE;
		}

		if (--_timeout == 0) {
			pif_error = E_TIMEOUT;
			return FALSE;
		}

//		pifTaskManager_Yield();
	}
	return TRUE;
}

static BOOL TWI_WaitByteSent(Twi *_twi, uint32_t _timeout)
{
	uint32_t _status_reg = 0;

	while ((_status_reg & TWI_SR_TXRDY) != TWI_SR_TXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK) {
			pif_error = E_RECEIVE_NACK;
			return FALSE;
		}

		if (--_timeout == 0) {
			pif_error = E_TIMEOUT;
			return FALSE;
		}

//		pifTaskManager_Yield();
	}

	return TRUE;
}

static BOOL TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout)
{
	uint32_t _status_reg = 0;

	while ((_status_reg & TWI_SR_RXRDY) != TWI_SR_RXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK) {
			pif_error = E_RECEIVE_NACK;
			return FALSE;
		}

		if (--_timeout == 0) {
			pif_error = E_TIMEOUT;
			return FALSE;
		}

//		pifTaskManager_Yield();
	}

	return TRUE;
}

void I2C_Init(uint32_t clock)
{
	twi = WIRE_INTERFACE;
	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SDA].pPort,
			g_APinDescription[PIN_WIRE_SDA].ulPinType,
			g_APinDescription[PIN_WIRE_SDA].ulPin,
			g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE_SCL].pPort,
			g_APinDescription[PIN_WIRE_SCL].ulPinType,
			g_APinDescription[PIN_WIRE_SCL].ulPin,
			g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);
	NVIC_SetPriority(WIRE_ISR_ID, 0);
	NVIC_EnableIRQ(WIRE_ISR_ID);

	// Disable PDC channel
	twi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

	TWI_ConfigureMaster(twi, clock, VARIANT_MCK);
}

void I2C_Exit()
{
	TWI_Disable(twi);

	// Enable PDC channel
	twi->TWI_PTCR &= ~(UART_PTCR_RXTDIS | UART_PTCR_TXTDIS);

	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);

	pmc_disable_periph_clk(WIRE_INTERFACE_ID);

	// no need to undo PIO_Configure,
	// as Peripheral A was enable by default before,
	// and pullups were not enabled
}

void I2C_XmitTimeout(uint32_t timeout)
{
	xmit_timeout = timeout;
}

void I2C_RecvTimeout(uint32_t timeout)
{
	recv_timeout = timeout;
}

BOOL I2C_ReadAddr(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint8_t size)
{
	int readed = 0;

	TWI_StartRead(twi, addr, iaddr, isize);
	do {
		// Stop condition must be set during the reception of last byte
		if (readed + 1 == size)
			TWI_SendSTOPCondition( twi);

		if (TWI_WaitByteReceived(twi, recv_timeout))
			p_data[readed++] = TWI_ReadByte(twi);
		else
			break;
	}
	while (readed < size);
	if (!TWI_WaitTransferComplete(twi, recv_timeout)) return FALSE;
	return readed >= size;
}

BOOL I2C_Read(uint8_t addr, uint8_t* p_data, uint8_t size)
{
	return I2C_ReadAddr(addr, 0, 0, p_data, size);
}

BOOL I2C_WriteAddr(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint8_t size)
{
	pif_error = E_SUCCESS;

	TWI_StartWrite(twi, addr, iaddr, isize, p_data[0]);
	TWI_WaitByteSent(twi, xmit_timeout);

	if (pif_error == E_SUCCESS) {
		uint16_t sent = 1;
		while (sent < size) {
			TWI_WriteByte(twi, p_data[sent++]);
			if (!TWI_WaitByteSent(twi, xmit_timeout)) break;
		}
	}

	if (pif_error == E_SUCCESS) {
		TWI_Stop(twi);
		TWI_WaitTransferComplete(twi, xmit_timeout);
	}
	return pif_error == E_SUCCESS;
}

BOOL I2C_Write(uint8_t addr, uint8_t* p_data, uint8_t size)
{
	return I2C_WriteAddr(addr, 0, 0, p_data, size);
}

#endif


PifI2cReturn actI2cWrite(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	uint8_t error;
#ifdef USE_I2C_WIRE
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
#else
	if (!I2C_WriteAddr(addr, iaddr, isize, p_data, size)) {
		error = pif_error;
		goto fail;
	}
#endif
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
#ifdef USE_I2C_WIRE
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
#else
	if (!I2C_ReadAddr(addr, iaddr, isize, p_data, size)) {
		error = pif_error;
		goto fail;
	}
#endif
    return IR_COMPLETE;

fail:
#ifndef __PIF_NO_LOG__
	pifLog_Printf(LT_ERROR, "I2CR(%Xh) IA=%lXh IS=%u S=%u E=%u", addr, iaddr, isize, size, error);
#endif
	return IR_ERROR;
}
