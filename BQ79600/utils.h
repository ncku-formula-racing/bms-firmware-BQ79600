#pragma once

#include <math.h>

#define B_vaule 3435.0f
#define V_max 5000.0f

#define NTC2T(mV) (1 / (log((mV) / (V_max - (mV))) / B_vaule + 1 / 298.15f) - 273.15f)
