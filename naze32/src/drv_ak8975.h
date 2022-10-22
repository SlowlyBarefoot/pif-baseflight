#pragma once

bool ak8975detect(sensorSet_t *p_sensor_set, void* p_param);
BOOL ak8975Init(PifImuSensorAlign align);
void ak8975Read(int16_t *magData);
