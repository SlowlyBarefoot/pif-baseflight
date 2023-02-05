/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "main.h"
#include "board.h"
#include "link_driver.h"
#include "drv_i2c.h"
#include "drv_system.h"


#ifdef USE_I2C_POLLING

PifI2cReturn actI2cRead(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	if (isize) {
		return (HAL_I2C_Mem_Read(&hi2c1, (addr << 1) | 1, iaddr, isize, p_data, size, 1000) == HAL_OK) ? IR_COMPLETE : IR_ERROR;
	}
	else {
		return (HAL_I2C_Master_Receive(&hi2c1, (addr << 1) | 1, p_data, size, 1000) == HAL_OK) ? IR_COMPLETE : IR_ERROR;
	}
}

PifI2cReturn actI2cWrite(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	if (isize) {
		return (HAL_I2C_Mem_Write(&hi2c1, addr << 1, iaddr, isize, p_data, size, 1000) == HAL_OK) ? IR_COMPLETE : IR_ERROR;
	}
	else {
		return (HAL_I2C_Master_Transmit(&hi2c1, addr << 1, p_data, size, 1000) == HAL_OK) ? IR_COMPLETE : IR_ERROR;
	}
}

#endif

#ifdef USE_I2C_INTERRUPT

PifI2cReturn actI2cRead(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	if (isize) {
		return (HAL_I2C_Mem_Read_IT(&hi2c1, (addr << 1) | 1, iaddr, isize, p_data, size) == HAL_OK) ? IR_WAIT : IR_ERROR;
	}
	else {
		return (HAL_I2C_Master_Receive_IT(&hi2c1, (addr << 1) | 1, p_data, size) == HAL_OK) ? IR_WAIT : IR_ERROR;
	}
}

PifI2cReturn actI2cWrite(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	if (isize) {
		return (HAL_I2C_Mem_Write_IT(&hi2c1, addr << 1, iaddr, isize, p_data, size) == HAL_OK) ? IR_WAIT : IR_ERROR;
	}
	else {
		return (HAL_I2C_Master_Transmit_IT(&hi2c1, addr << 1, p_data, size) == HAL_OK) ? IR_WAIT : IR_ERROR;
	}
}

#endif

#ifdef USE_I2C_DMA

PifI2cReturn actI2cRead(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	if (isize) {
		return (HAL_I2C_Mem_Read_DMA(&hi2c1, (addr << 1) | 1, iaddr, isize, p_data, size) == HAL_OK) ? IR_WAIT : IR_ERROR;
	}
	else {
		return (HAL_I2C_Master_Receive_DMA(&hi2c1, (addr << 1) | 1, p_data, size) == HAL_OK) ? IR_WAIT : IR_ERROR;
	}
}

PifI2cReturn actI2cWrite(uint8_t addr, uint32_t iaddr, uint8_t isize, uint8_t* p_data, uint16_t size)
{
	if (isize) {
		return (HAL_I2C_Mem_Write_DMA(&hi2c1, addr << 1, iaddr, isize, p_data, size) == HAL_OK) ? IR_WAIT : IR_ERROR;
	}
	else {
		return (HAL_I2C_Master_Transmit_DMA(&hi2c1, addr << 1, p_data, size) == HAL_OK) ? IR_WAIT : IR_ERROR;
	}
}

#endif

#if defined(USE_I2C_INTERRUPT) || defined(USE_I2C_DMA)

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {
		pifI2cPort_sigEndTransfer(&g_i2c_port, TRUE);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {
		pifI2cPort_sigEndTransfer(&g_i2c_port, TRUE);
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {
		pifI2cPort_sigEndTransfer(&g_i2c_port, TRUE);
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {
		pifI2cPort_sigEndTransfer(&g_i2c_port, TRUE);
	}
}

#endif

BOOL i2cInit()
{
    if (!pifI2cPort_Init(&g_i2c_port, PIF_ID_AUTO, 5, EEPROM_PAGE_SIZE)) return FALSE;
    g_i2c_port.act_read = actI2cRead;
    g_i2c_port.act_write = actI2cWrite;
    return TRUE;
}
