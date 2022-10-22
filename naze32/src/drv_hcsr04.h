#pragma once

#include "board.h"
#include "mw.h"

#ifdef __cplusplus
extern "C" {
#endif

BOOL hcsr04Init(uint16_t period, sonarDistanceFuncPtr func);	// period unit : ms

#ifdef __cplusplus
}
#endif
