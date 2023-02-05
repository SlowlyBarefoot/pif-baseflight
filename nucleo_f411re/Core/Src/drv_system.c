/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "main.h"
#include "board.h"
#include "link_driver.h"

#include "drv_hcsr04.h"
#include "drv_pwm.h"
#include "drv_system.h"

#include "core/pif_log.h"
#include "storage/pif_storage_fix.h"
#include "storage/pif_storage_var.h"


#ifdef STORAGE_FLASH

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_6   /* Start @ of user Flash area */

#endif


#ifdef STORAGE_FLASH
	PifStorageFix s_storage;
#endif
#ifdef STORAGE_EEPROM
	PifStorageVar s_storage;
#endif


void systemInit(void)
{
#ifdef BUZZER
    actBuzzerAction(PIF_ID_BUZZER, OFF);
#endif
    actLed0State(OFF);
    actLed1State(OFF);
}

void failureMode(uint8_t mode)
{
#ifdef __PIF_DEBUG__
    pifLog_SendAndExit();
#endif
    actLed1State(OFF);
    actLed0State(ON);
    while (1) {
        actLed1Toggle();
        actLed0Toggle();
        pif_Delay1ms(475 * mode - 2);
        actBuzzerAction(PIF_ID_BUZZER, ON);
        pif_Delay1ms(25);
        actBuzzerAction(PIF_ID_BUZZER, OFF);
    }
}

void systemReset(bool toBootloader)
{
	pifLog_Printf(LT_INFO, "System Rebooting %d...", toBootloader);
	pif_Delay1ms(2000);
//	__DSB;
	SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);//software reset
	//RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;
	//NVIC_SystemReset();
}

static BOOL len0_state = OFF;

void actLed0State(BOOL state)
{
#ifdef LED0
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, state);
    len0_state = state;
#else
    (void)state;
#endif
}

void actLed0Toggle()
{
#ifdef LED0
	len0_state ^= 1;
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, len0_state);
#endif
}

static BOOL len1_state = OFF;

void actLed1State(BOOL state)
{
#ifdef LED1
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, state);
    len1_state = state;
#else
    (void)state;
#endif
}

void actLed1Toggle()
{
#ifdef LED1
	len1_state ^= 1;
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, len1_state);
#endif
}

void actBuzzerAction(PifId id, BOOL action)
{
	(void)id;

#ifdef BUZZER
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, action);
#else
    (void)action;
#endif
}

void actInvState(BOOL state)
{
#ifdef INVERTER
	HAL_GPIO_WritePin(INVERTER_GPIO_Port, INVERTER_Pin, state);
#else
    (void)state;
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	extern void hcsr04Echo();

    switch (GPIO_Pin) {
    case RC_PPM_Pin:
    	pwmReadRc();
		break;

    case SONAR_ECHO_Pin:
    	hcsr04Echo();
    	break;
    }
}

#ifdef STORAGE_FLASH

static uint32_t GetSector(uint32_t address)
{
	uint32_t sector = 0;

	if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0)) {
		sector = FLASH_SECTOR_0;	// 16Kbytes
	}
	else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1)) {
		sector = FLASH_SECTOR_1;	// 16Kbytes
	}
	else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2)) {
		sector = FLASH_SECTOR_2;	// 16Kbytes
	}
	else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3)) {
		sector = FLASH_SECTOR_3;	// 16Kbytes
	}
	else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4)) {
		sector = FLASH_SECTOR_4;	// 64Kbytes
	}
	else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5)) {
		sector = FLASH_SECTOR_5;	// 128Kbytes
	}
	else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6)) {
		sector = FLASH_SECTOR_6;	// 128Kbytes
	}
	else {
		sector = FLASH_SECTOR_7;
	}

	return sector;
}

static HAL_StatusTypeDef EraseFlash(uint32_t start_addr)
{
	uint32_t sector_error = 0;

	/* Unlock to control */
	HAL_FLASH_Unlock();

	/* Calculate sector index */

	/* Erase sectors */
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = GetSector(start_addr);
	EraseInitStruct.NbSectors = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error) != HAL_OK) return HAL_ERROR;

	/* Clear cache for flash */
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	/* Lock flash control register */
	HAL_FLASH_Lock();

	return HAL_OK;
}

BOOL actStorageRead(PifStorage* p_owner, uint8_t* dst, uint32_t src, size_t size)
{
	uint32_t address = FLASH_USER_START_ADDR + src;
	uint32_t* dp = (uint32_t*)dst;
	size_t i;

	(void)p_owner;

	for (i = 0; i < size; i += 4) {
		*dp = *(__IO uint32_t*)(address + i);
		dp++;
	}
	return TRUE;
}

BOOL actStorageWrite(PifStorage* p_owner, uint32_t dst, uint8_t* src, size_t size)
{
	uint32_t address = FLASH_USER_START_ADDR + dst;
	uint32_t* sp = (uint32_t*)src;
	size_t i;

	(void)p_owner;

	if (EraseFlash(address) != HAL_OK) return FALSE;

	/* Unlock to control */
	HAL_FLASH_Unlock();

	for (i = 0; i < size; i += 4) {
		/* Writing data to flash memory */
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, *sp) != HAL_OK) {
			return FALSE;
		}
		sp++;
	}

	/* Lock flash control register */
	HAL_FLASH_Lock();

	return TRUE;
}

#endif

PifStorage* storageInit()
{
#ifdef STORAGE_FLASH
	if (!pifStorageFix_Init(&s_storage, PIF_ID_AUTO)) return NULL;
	if (!pifStorageFix_AttachActStorage(&s_storage, actStorageRead, actStorageWrite)) goto fail;
	if (!pifStorageFix_SetMedia(&s_storage, STORAGE_SECTOR_SIZE, STORAGE_VOLUME)) goto fail;
#endif
#ifdef STORAGE_EEPROM
	if (!pifStorageVar_Init(&s_storage, PIF_ID_AUTO)) return NULL;
	if (!pifStorageVar_AttachI2c(&s_storage, &g_i2c_port, ATMEL_I2C_ADDRESS, EEPROM_I_ADDR_SIZE, 10)) goto fail;	// 10ms
	if (!pifStorageVar_SetMedia(&s_storage, STORAGE_SECTOR_SIZE, STORAGE_VOLUME, 8)) goto fail;
	if (!pifStorageVar_IsFormat(&s_storage.parent)) {
		pifLog_Printf(LT_INFO, "Storage Init : EC=%d", pif_error);
		if (!pifStorage_Format(&s_storage.parent)) {
			pifLog_Printf(LT_INFO, "Storage format failed");
			goto fail;
		}
		else {
			pifLog_Printf(LT_INFO, "Storage format");
		}
	}
#endif
	return &s_storage.parent;

fail:
#ifdef STORAGE_FLASH
	pifStorageFix_Clear(&s_storage);
#endif
#ifdef STORAGE_EEPROM
	pifStorageVar_Clear(&s_storage);
#endif
	return NULL;
}
