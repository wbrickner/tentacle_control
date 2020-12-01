#pragma once

#include "sensor.h"
#include "stepper.h"

// given maximum displacement, how much should the corresponding phase space values change?
// I mean this literally, the phase space values in this case would be adjusted by exactly this amount.
#define RELAXATION_COEFFICIENT 0.33

typedef struct Displacement { signed char dx; signed char dy; } Displacement;
extern Displacement displacement;

void maximaDisplacement();
void seekAlignment();