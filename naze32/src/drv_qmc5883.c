#include "board.h"
#include "link_driver.h"

#include "drv_qmc5883.h"

#include "sensor/pif_qmc5883.h"

static PifQmc5883 qmc5883;

static const char* hw_names = "QMC5883";

static BOOL qmc5883lInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align);
static BOOL qmc5883lRead(sensorSet_t *p_sensor_set, int16_t *magData);

bool qmc5883Detect(sensorSet_t *p_sensor_set, void* p_param)
{
    (void)p_param;

    if (!pifQmc5883_Init(&qmc5883, PIF_ID_AUTO, &g_i2c_port, &p_sensor_set->imu_sensor)) return false;

    p_sensor_set->mag.hardware = hw_names;
    p_sensor_set->mag.init = qmc5883lInit;
    p_sensor_set->mag.read = qmc5883lRead;
    return true;
}

static BOOL qmc5883lInit(sensorSet_t *p_sensor_set, PifImuSensorAlign align)
{
    PifQmc5883Control1 control_1;

    pifImuSensor_SetMagAlign(&p_sensor_set->imu_sensor, align);

    control_1.bit.mode = QMC5883_MODE_CONTIMUOUS;
    control_1.bit.odr = QMC5883_ODR_10HZ;
    control_1.bit.rng = QMC5883_RNG_2G;
    control_1.bit.osr = QMC5883_OSR_64;
    pifQmc5883_SetControl1(&qmc5883, control_1);
    return TRUE;
}

static BOOL qmc5883lRead(sensorSet_t *p_sensor_set, int16_t *magData)
{
    return pifImuSensor_ReadMag2(&p_sensor_set->imu_sensor, magData);
}
