#pragma once

typedef struct drv_adc_config_t {
    uint8_t powerAdcChannel;     // which channel used for current monitor, allowed PA1, PB1 (ADC_Channel_1, ADC_Channel_9)
    uint8_t rssiAdcChannel;      // which channel used for analog-rssi (RC-filter), allowed PA1, PB1 (ADC_Channel_1, ADC_Channel_9)
} drv_adc_config_t;

void adcInit(drv_adc_config_t *init);

