#pragma once

#define BKP_SOFTRESET (0x50F7B007)

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }

// flash total size
#define FLASH_SIZE                      0x20000

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
// if sizeof(mcfg) is over this number, compile-time error will occur. so, need to add another page to config data.
#define CONFIG_SIZE                     (FLASH_PAGE_SIZE * 2)

#define STORAGE_SECTOR_SIZE		        FLASH_PAGE_SIZE
#define STORAGE_VOLUME			        (FLASH_PAGE_SIZE * 4)

void systemInit(void);

uint32_t micros(void);

// Backup SRAM R/W
uint32_t rccReadBkpDr(void);
void rccWriteBkpDr(uint32_t value);

void productionDebug(void);

// current crystal frequency - 8 or 12MHz
extern uint32_t hse_value;

