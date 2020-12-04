#pragma once

#include "sensor.h"
#include "stepper.h"

// given maximum displacement, how much should the corresponding phase space values change?
// I mean this literally, the phase space values in this case would be adjusted by exactly this amount.
#define RELAXATION_COEFFICIENT 0.5

/*
 * Coordinate system:
 *                [ARM UP]
 * 
 *                   -y
 *                    |
 * [ARM LEFT]   -x ---|--- +x   [ARM RIGHT]
 *                    |
 *                   +y
 * 
 *               [ARM DOWN]
 */
typedef struct Displacement { float dx; float dy; } Displacement;
extern Displacement displacement;

void centroidDisplacement();
void seekAlignment();