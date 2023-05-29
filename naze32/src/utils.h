#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifndef constrain
__attribute__ ((weak)) int constrain(int amt, int low, int high);
#endif
// sensor orientation
void alignSensors(int16_t *src, float *dest, uint8_t rotation);
void initBoardAlignment(void);

#ifdef __cplusplus
}
#endif

