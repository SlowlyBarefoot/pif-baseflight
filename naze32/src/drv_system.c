/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "link_driver.h"

#include "drv_gpio.h"
#include "drv_system.h"

#include "storage/pif_storage_var.h"

static const uint32_t FLASH_WRITE_ADDR = 0x08000000 + FLASH_SIZE - STORAGE_VOLUME;

// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;

PifStorageVar s_storage;

#ifdef BUZZER
void systemBeep(bool onoff);
static void beepRev4(bool onoff);
static void beepRev5(bool onoff);
void (*systemBeepPtr)(bool onoff) = NULL;
#endif

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;

    pif_sigTimer1ms();
	pifTimerManager_sigTick(&g_timer_1ms);
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void systemInit(void)
{
    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup[] = {
#ifdef LED0
        {
            .gpio = LED0_GPIO,
            .cfg = { LED0_PIN, Mode_Out_PP, Speed_2MHz }
        },
#endif
#ifdef LED1
        {
            .gpio = LED1_GPIO,
            .cfg = { LED1_PIN, Mode_Out_PP, Speed_2MHz }
        },
#endif
#ifdef BUZZER
        {
            .gpio = BEEP_GPIO,
            .cfg = { BEEP_PIN, Mode_Out_OD, Speed_2MHz }
        },
#endif
#ifdef INVERTER
        {
            .gpio = INV_GPIO,
            .cfg = { INV_PIN, Mode_Out_PP, Speed_2MHz }
        },
#endif
    };
    gpio_config_t gpio;
    int i, gpio_count = sizeof(gpio_setup) / sizeof(gpio_setup[0]);

    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    gpio.pin = Pin_All;
    gpio.mode = Mode_AIN;
    gpioInit(GPIOA, &gpio);
    gpioInit(GPIOB, &gpio);
    gpioInit(GPIOC, &gpio);

    // Turn off JTAG port 'cause we're using the GPIO for leds
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;

#ifdef BUZZER
    // Configure gpio
    // rev5 needs inverted beeper. oops.
    if (hw_revision >= NAZE32_REV5)
        systemBeepPtr = beepRev5;
    else
        systemBeepPtr = beepRev4;
    actBuzzerAction(PIF_ID_BUZZER, OFF);
#endif
    actLed0State(OFF);
    actLed1State(OFF);

    // Hack - rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.
    for (i = 0; i < gpio_count; i++) {
        if (hw_revision >= NAZE32_REV5 && gpio_setup[i].cfg.mode == Mode_Out_OD)
            gpio_setup[i].cfg.mode = Mode_Out_PP;
        gpioInit(gpio_setup[i].gpio, &gpio_setup[i].cfg);
    }

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}

void failureMode(uint8_t mode)
{
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

uint32_t rccReadBkpDr(void)
{
    return *((uint16_t *)BKP_BASE + 0x04) | *((uint16_t *)BKP_BASE + 0x08) << 16;
}

void rccWriteBkpDr(uint32_t value)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR->CR |= PWR_CR_DBP;

    *((uint16_t *)BKP_BASE + 0x04) = value & 0xffff;
    *((uint16_t *)BKP_BASE + 0x08) = (value & 0xffff0000) >> 16;
}

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    if (toBootloader) {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC
        *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
    }

    // write magic value that we're doing a soft reset
    rccWriteBkpDr(BKP_SOFTRESET);

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

#ifdef BUZZER
static void beepRev4(bool onoff)
{
    if (onoff) {
        digitalLo(BEEP_GPIO, BEEP_PIN);
    } else {
        digitalHi(BEEP_GPIO, BEEP_PIN);
    }
}

static void beepRev5(bool onoff)
{
    if (onoff) {
        digitalHi(BEEP_GPIO, BEEP_PIN);
    } else {
        digitalLo(BEEP_GPIO, BEEP_PIN);
    }
}

void systemBeep(bool onoff)
{
    systemBeepPtr(onoff);
}
#endif

void actLed0State(BOOL state)
{
#ifdef LED0
    if (state) {
        digitalLo(LED0_GPIO, LED0_PIN); 
    } 
    else {
        digitalHi(LED0_GPIO, LED0_PIN);
    }
#else
    (void)state;
#endif
}

void actLed0Toggle()
{
#ifdef LED0
    digitalToggle(LED0_GPIO, LED0_PIN);
#endif
}

void actLed1State(BOOL state)
{
#ifdef LED1
    if (state) {
        digitalLo(LED1_GPIO, LED1_PIN); 
    } 
    else {
        digitalHi(LED1_GPIO, LED1_PIN);
    }
#else
    (void)state;
#endif
}

void actLed1Toggle()
{
#ifdef LED1
    digitalToggle(LED1_GPIO, LED1_PIN);
#endif
}

void actBuzzerAction(PifId id, BOOL action)
{
	(void)id;

#ifdef BEEP_GPIO
    if (action) {
        systemBeep(true); 
    } 
    else {
        systemBeep(false);
    }
#else
    (void)action;
#endif
}

void actInvState(BOOL state)
{
#ifdef INV_GPIO
    if (state) {
        digitalHi(INV_GPIO, INV_PIN); 
    } 
    else {
        digitalLo(INV_GPIO, INV_PIN);
    }
#else
    (void)state;
#endif
}

BOOL actStorageRead(PifStorage* p_owner, uint8_t* dst, uint32_t src, size_t size)
{
	(void)p_owner;

    memcpy(dst, (char *)FLASH_WRITE_ADDR + src, size);
	return TRUE;
}

BOOL actStorageWrite(PifStorage* p_owner, uint32_t dst, uint8_t* src, size_t size)
{
    FLASH_Status status;

	(void)p_owner;

    FLASH_Unlock();
    for (unsigned int tries = 3; tries; tries--) {
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

        status = FLASH_ErasePage(FLASH_WRITE_ADDR + dst);
        for (unsigned int i = 0; i < size && status == FLASH_COMPLETE; i += 4)
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + dst + i, *(uint32_t *)(src + i));
        if (status == FLASH_COMPLETE)
            break;
    }
    FLASH_Lock();

    return status == FLASH_COMPLETE;
}

PifStorage* storageInit()
{
	if (!pifStorageVar_Init(&s_storage, PIF_ID_AUTO)) return NULL;
	if (!pifStorageVar_AttachActStorage(&s_storage, actStorageRead, actStorageWrite)) goto fail;
	if (!pifStorageVar_SetMedia(&s_storage, STORAGE_SECTOR_SIZE, STORAGE_VOLUME, 8)) goto fail;
	return &s_storage.parent;

fail:
	pifStorageVar_Clear(&s_storage);
	return NULL;
}
#ifdef PROD_DEBUG
void productionDebug(void)
{
    gpio_config_t gpio;

    // remap PB6 to USART1_TX
    gpio.pin = Pin_6;
    gpio.mode = Mode_AF_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(GPIOB, &gpio);
    gpioPinRemapConfig(AFIO_MAPR_USART1_REMAP, true);
    serialInit(mcfg.serial_baudrate, hw_revision == NAZE32_SP && !mcfg.spektrum_sat_on_flexport);
    pif_Delay1ms(25);
    serialPrint(core.mainport, "DBG ");
    printf("%08x%08x%08x OK\n", U_ID_0, U_ID_1, U_ID_2);
    serialPrint(core.mainport, "EOF");
    pif_Delay1ms(25);
    gpioPinRemapConfig(AFIO_MAPR_USART1_REMAP, false);
}
#endif
