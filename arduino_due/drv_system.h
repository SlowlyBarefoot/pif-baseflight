#pragma once

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }

//#define STORAGE_FLASH
#define STORAGE_EEPROM

#ifdef STORAGE_FLASH
	#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
	#define STORAGE_SECTOR_SIZE		        FLASH_PAGE_SIZE
	#define STORAGE_VOLUME			        (FLASH_PAGE_SIZE * 4)
#endif
#ifdef STORAGE_EEPROM
	#define ATMEL_I2C_ADDRESS		0x50
	#define EEPROM_I_ADDR_SIZE		SIC_I_ADDR_SIZE_2
	#define EEPROM_PAGE_SIZE		64
	#define STORAGE_SECTOR_SIZE		64
	#define STORAGE_VOLUME			32768
	#define I2C_TRANSFER_SIZE		24
#else
	#define I2C_TRANSFER_SIZE		16
#endif


#ifdef __cplusplus
extern "C" {
#endif

void systemInit(void);

// Inv
void actInvState(BOOL state);

// storage
PifStorage* storageInit();

#ifdef __cplusplus
}
#endif
