#pragma once

#include <AccelStepper.h>
#include <MultiStepper.h>

#define ACCELERATION 5000  // (steps / second) / second
#define MAX_SPEED    600   // steps / second

// the global abstract motor state
typedef struct OrientationState { float xa; float ya; float xb; float yb; } OrientationState;
extern OrientationState motorState;

void initMotors();
void updateOrientations();
void runSteppers();