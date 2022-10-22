#pragma once

bool hmc5883lDetect(sensorSet_t *p_sensor_set, void* p_param);
BOOL hmc5883lInit(PifImuSensorAlign align);
void hmc5883lRead(int16_t *magData);
