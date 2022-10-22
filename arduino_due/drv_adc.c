#include "board.h"
#include "mw.h"

#include "drv_adc.h"

// Driver for STM32F103CB onboard ADC
// VBAT is connected to PA4 (ADC1_IN4) with 10k:1k divider
// rev.5 hardware has PA5 (ADC1_IN5) on breakout pad on bottom of board
// Additional channel can be stolen from RC_CH2 (PA1, ADC1_IN1) or
// RC_CH8 (PB1, ADC1_IN9) by using set power_adc_channel=1|9

static uint8_t adcChannel[ADC_CHANNEL_MAX];

void adcInit(drv_adc_config_t *init)
{
	adcChannel[ADC_BATTERY] = A0;
	adcChannel[ADC_EXTERNAL_CURRENT] = A1;

    if (init->rssiAdcChannel > 0) {
        adcChannel[ADC_RSSI] = A2;
    }
}

uint16_t actGetAdcChannel(uint8_t channel)
{
	return analogRead(adcChannel[channel]);
}

float actGetBatteryVoltage()
{
	return (analogRead(adcChannel[ADC_BATTERY]) * 3.3f) / 4095;
}

#define ADCVREF 33L
uint32_t actGetBatteryCurrent()
{
    return ((uint32_t)analogRead(adcChannel[ADC_EXTERNAL_CURRENT]) * ADCVREF * 100) / 4095;
}
