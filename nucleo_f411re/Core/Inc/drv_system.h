#pragma once

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }

//#define STORAGE_FLASH
#define STORAGE_EEPROM

#ifdef STORAGE_FLASH
	// flash total size
	#define FLASH_SIZE                      0x20000

	#define FLASH_PAGE_SIZE                 ((uint16_t)64)
	// if sizeof(mcfg) is over this number, compile-time error will occur. so, need to add another page to config data.
	#define CONFIG_SIZE                     (FLASH_PAGE_SIZE * 16)

	#define STORAGE_SECTOR_SIZE		        FLASH_PAGE_SIZE
	#define STORAGE_VOLUME			        1024
#endif
#ifdef STORAGE_EEPROM
	#define ATMEL_I2C_ADDRESS		0x50
	#define EEPROM_I_ADDR_SIZE		SIC_I_ADDR_SIZE_2
	#define EEPROM_PAGE_SIZE		64
	#define STORAGE_SECTOR_SIZE		64
	#define STORAGE_VOLUME			32768
	#define I2C_TRANSFER_SIZE		EEPROM_PAGE_SIZE
#else
	#define I2C_TRANSFER_SIZE		16
#endif

void systemInit(void);

// Inv
void actInvState(BOOL state);
