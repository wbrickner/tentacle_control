#include "stepper.h"

AccelStepper xa(2, 10, 11); AccelStepper ya(2, 4, 5);
AccelStepper xb(2,  8, 9); AccelStepper yb(2, 6, 7);

// ordering for the following two objects are [ xa, ya, xb, yb ]
MultiStepper steppers;
long locations[4] = { 0, 0, 0, 0 };

OrientationState motorState = { xa: 0.0, ya: 0.0, xb: 0.0, yb: 0.0 };

// configures a motor in isolation
void initMotor(AccelStepper& s) {
  s.setAcceleration(ACCELERATION);
  s.setMaxSpeed(MAX_SPEED);
}

// initializes all motors. WARN: only call once.
void initMotors() {
  initMotor(xa); initMotor(ya);
  initMotor(xb); initMotor(yb);

  steppers.addStepper(xa); steppers.addStepper(ya);
  steppers.addStepper(xb); steppers.addStepper(yb);
}

long toStepsSafe(float abstract) { 
  return (long)(100.0 * abstract);
}

// updates all stepper motor target positions
void updateOrientations() {
  // transform abstract position to step position
  locations[0] = toStepsSafe(motorState.xa);
  locations[1] = toStepsSafe(motorState.ya);
  locations[2] = toStepsSafe(motorState.xb);
  locations[3] = toStepsSafe(motorState.yb);

  // update the target locations stored in the multistepper
  steppers.moveTo(locations);
}

// moves all motors by some ∆u_i, each seeking their position with acceleration.
void runSteppers() { steppers.run(); }