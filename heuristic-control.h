#pragma once

#include "sensor.h"
#include "stepper.h"

typedef struct Displacement { signed char dx; signed char dy; } Displacement;
extern Displacement displacement;

void maximaDisplacement();
void seekAlignment();