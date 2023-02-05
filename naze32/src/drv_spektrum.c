/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "link_driver.h"

#include "drv_gpio.h"
#include "drv_system.h"


// driver for spektrum satellite receiver / sbus using UART2 (freeing up more motor outputs for stuff)

static GPIO_TypeDef *spekBindPort = NULL;
static uint16_t spekBindPin = 0;

/* spektrumBind function. It's used to bind satellite receiver to TX.
 * Function must be called immediately after startup so that we don't miss satellite bind window.
 * Known parameters. Tested with DSMX satellite and DX8 radio. Framerate (11ms or 22ms) must be selected from TX.
 * 9 = DSMX 11ms / DSMX 22ms
 * 5 = DSM2 11ms 2048 / DSM2 22ms 1024
 */
int spektrumBind(uint8_t spektrum_sat_on_flexport, uint8_t spektrum_sat_bind)
{
    int i;
    gpio_config_t gpio;
    int spekUart = 0;

#ifdef HARDWARE_BIND_PLUG
    // Check status of bind plug and exit if not active
    GPIO_TypeDef *hwBindPort = NULL;
    uint16_t hwBindPin = 0;

    hwBindPort = GPIOB;
    hwBindPin = Pin_5;
    gpio.speed = Speed_2MHz;
    gpio.pin = hwBindPin;
    gpio.mode = Mode_IPU;
    gpioInit(hwBindPort, &gpio);
    if (digitalIn(hwBindPort, hwBindPin))
        return 0;
#endif

    if (spektrum_sat_on_flexport) {
        // USART3, PB11
        spekBindPort = GPIOB;
        spekBindPin = Pin_11;
        spekUart = 3;
    } else {
        // USART2, PA3
        spekBindPort = GPIOA;
        spekBindPin = Pin_3;
        spekUart = 2;
    }

    // don't try to bind if: here after soft reset or bind flag is out of range
    if (rccReadBkpDr() == BKP_SOFTRESET || spektrum_sat_bind == 0 || spektrum_sat_bind > 10)
        return 0;

    gpio.speed = Speed_2MHz;
    gpio.pin = spekBindPin;
    gpio.mode = Mode_Out_OD;
    gpioInit(spekBindPort, &gpio);
    // RX line, set high
    digitalHi(spekBindPort, spekBindPin);
    // Bind window is around 20-140ms after powerup
    pif_Delay1ms(60);

    for (i = 0; i < spektrum_sat_bind; i++) {
        // RX line, drive low for 120us
        digitalLo(spekBindPort, spekBindPin);
        pif_Delay1us(120);
        // RX line, drive high for 120us
        digitalHi(spekBindPort, spekBindPin);
        pif_Delay1us(120);
    }
    return spekUart;
}
