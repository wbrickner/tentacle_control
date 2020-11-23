#pragma once

#include <AccelStepper.h>

#define ACCELERATION 4000
#define MAX_SPEED 200

// the global abstract motor state
typedef struct OrientationState { float xa; float ya; float xb; float yb; } OrientationState;

extern OrientationState motorState;
extern AccelStepper xa, ya, xb, yb;

void initMotor(AccelStepper& s);
void updateOrientations();
void runSteppers();
void initMotors();