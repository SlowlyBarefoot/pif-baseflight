#pragma once

#include "board.h"
#include "link_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

BOOL hcsr04Init(uint16_t period, sonarDistanceFuncPtr func);	// period unit : ms

#ifdef __cplusplus
}
#endif
