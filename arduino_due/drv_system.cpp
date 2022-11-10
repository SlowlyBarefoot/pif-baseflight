/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"

#include "drv_system.h"

#include "core/pif_log.h"
#include "storage/pif_storage_var.h"

#include <DueFlashStorage.h>


#define LED0_PIN    30
#define LED1_PIN    31
#define BEEP_PIN    32
#define INV_PIN     33


static DueFlashStorage dueFlashStorage;

PifStorageVar s_storage;


void systemInit(void)
{
	pinMode(LED0_PIN, OUTPUT);
	pinMode(LED1_PIN, OUTPUT);
	pinMode(BEEP_PIN, OUTPUT);
	pinMode(INV_PIN, OUTPUT);

#ifdef BUZZER
    actBuzzerAction(PIF_ID_BUZZER, OFF);
#endif
    actLed0State(OFF);
    actLed1State(OFF);
}

void failureMode(uint8_t mode)
{
	pifLog_Print(LT_ERROR, "Failure mode");
    actLed1State(OFF);
    actLed0State(ON);
    pifLog_SendAndExit();
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
	(void)toBootloader;

	pif_Delay1ms(2000);

	SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);//software reset
}

static BOOL len0_state = OFF;

void actLed0State(BOOL state)
{
#ifdef LED0
    if (state) {
        digitalWrite(LED0_PIN, ON);
    } 
    else {
    	digitalWrite(LED0_PIN, OFF);
    }
    len0_state = state;
#else
    (void)state;
#endif
}

void actLed0Toggle()
{
#ifdef LED0
	len0_state ^= 1;
	if (len0_state) {
        digitalWrite(LED0_PIN, ON);
    }
    else {
    	digitalWrite(LED0_PIN, OFF);
    }
#endif
}

static BOOL len1_state = OFF;

void actLed1State(BOOL state)
{
#ifdef LED1
    if (state) {
    	digitalWrite(LED1_PIN, ON);
    } 
    else {
    	digitalWrite(LED1_PIN, OFF);
    }
    len1_state = state;
#else
    (void)state;
#endif
}

void actLed1Toggle()
{
#ifdef LED1
	len1_state ^= 1;
    if (len1_state) {
    	digitalWrite(LED1_PIN, ON);
    }
    else {
    	digitalWrite(LED1_PIN, OFF);
    }
#endif
}

void actBuzzerAction(PifId id, BOOL action)
{
	(void)id;

#ifdef BUZZER
    if (action) {
    	digitalWrite(BEEP_PIN, ON);
    } 
    else {
    	digitalWrite(BEEP_PIN, OFF);
    }
#else
    (void)action;
#endif
}

void actInvState(BOOL state)
{
    if (state) {
    	digitalWrite(INV_PIN, ON);
    } 
    else {
    	digitalWrite(INV_PIN, OFF);
    }
}

#ifdef STORAGE_FLASH

BOOL actStorageRead(PifStorage* p_owner, uint8_t* dst, uint32_t src, size_t size)
{
	(void)p_owner;

	memcpy(dst, dueFlashStorage.readAddress(src), size);
	return TRUE;
}

BOOL actStorageWrite(PifStorage* p_owner, uint32_t dst, uint8_t* src, size_t size)
{
	(void)p_owner;

	return dueFlashStorage.write(dst, src, size);
}

#endif

PifStorage* storageInit()
{
	if (!pifStorageVar_Init(&s_storage, PIF_ID_AUTO)) return NULL;
#ifdef STORAGE_FLASH
	if (!pifStorageVar_AttachActStorage(&s_storage, actStorageRead, actStorageWrite)) goto fail;
#endif
#ifdef STORAGE_EEPROM
	if (!pifStorageVar_AttachI2c(&s_storage, &g_i2c_port, ATMEL_I2C_ADDRESS, EEPROM_I_ADDR_SIZE, 10)) goto fail;	// 10ms
#endif
	if (!pifStorageVar_SetMedia(&s_storage, STORAGE_SECTOR_SIZE, STORAGE_VOLUME, 8)) goto fail;
//	if (!pifStorageVar_Format(&s_storage.parent)) goto fail;
	return &s_storage.parent;

fail:
	pifStorageVar_Clear(&s_storage);
	return NULL;
}
