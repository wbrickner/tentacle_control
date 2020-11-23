#include "stepper.h"

OrientationState motorState = { xa: 0.0, ya: 0.0, xb: 0.0, yb: 0.0 };
AccelStepper xa(2, 2, 3); AccelStepper ya(2, 4, 5);
AccelStepper xb(2, 6, 7); AccelStepper yb(2, 8, 9);

// Sets new target orientation. Orientation is clamped to the safe range [-1, 1].
void updateOrientation(AccelStepper& s, float o) {
  float safe = max(min(o, 1.0), -1.0);
  // transform from floating point -> steps position; 
  // 200 steps total, so -1 = 0s = -90º, and +1 => 100s = +90º
  s.moveTo(50 * (safe + 1.0));
}

// moves all motors by some ∆x_i, each seeking their position with acceleration.
void runSteppers() {
  // we do not care if the motors actually reach their target position, 
  // as long as they make some progress towards it, so we advance them for 10ms
  unsigned long start = millis();
  while (millis() - start < 10) { xa.run(); ya.run(); xb.run(); yb.run(); }
}

// Updates all stepper motor target positions
void updateOrientations(OrientationState& state) {
  updateOrientation(xa, state.xa); updateOrientation(ya, state.ya);
  updateOrientation(xb, state.xb); updateOrientation(yb, state.yb);
}

// Configures a motor
void initMotor(AccelStepper& s) {
  s.setAcceleration(ACCELERATION);
  s.setMaxSpeed(MAX_SPEED);
}

void initMotors() {
  initMotor(xa); initMotor(ya);
  initMotor(xb); initMotor(yb);
}