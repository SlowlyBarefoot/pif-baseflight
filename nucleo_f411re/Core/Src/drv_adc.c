#include "main.h"
#include "board.h"
#include "link_driver.h"

#include "drv_adc.h"

// VBAT is connected to PA4 (ADC1_IN4) with 10k:1k divider

static uint32_t adcValues[ADC_CHANNEL_MAX];


void adcInit(drv_adc_config_t *init)
{
    extern ADC_HandleTypeDef hadc1;

    // Start ADC
    HAL_ADC_Start_DMA(&hadc1, adcValues, ADC_CHANNEL_MAX);
}

uint16_t actGetAdcChannel(uint8_t channel)
{
    return adcValues[channel];
}

float actGetBatteryVoltage()
{
	return (adcValues[ADC_BATTERY] * 3.3f) / 4095;
}

uint32_t actGetBatteryCurrent()
{
	return 5000 * adcValues[ADC_EXTERNAL_CURRENT] / 4095;
}
