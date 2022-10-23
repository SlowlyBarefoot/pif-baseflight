/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "mw.h"

#include "sensor/pif_bmp280.h"

static bool bmp280InitDone = false;
// uncompensated pressure and temperature
static PifBmp280 bmp280;

static const char* hw_names = "BMP280";

bool bmp280Detect(sensorSet_t *p_sensor_set, void* p_param)
{
    extern void evtBaroRead(int32_t pressure, float temperature);

    (void)p_param;

    if (bmp280InitDone)
        return true;

    pif_Delay1ms(20);

    if (!pifBmp280_Init(&bmp280, PIF_ID_AUTO, &g_i2c_port, BMP280_I2C_ADDR(0))) return false;

    // set oversampling + power mode (forced), and start sampling
    pifBmp280_SetOverSamplingRate(&bmp280, BMP280_OSRS_X8, BMP280_OSRS_X1);

    if (!pifBmp280_AddTaskForReading(&bmp280, 25, evtBaroRead)) return false;   // 25ms : 40hz update rate (20hz LPF on acc)
    bmp280.__p_task->disallow_yield_id = DISALLOW_YIELD_ID_I2C;

    bmp280InitDone = true;

    p_sensor_set->baro.hardware = hw_names;
    return true;
}
