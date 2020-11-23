#include <AccelStepper.h>
#include "sensor.h"
#include "stepper.h"
#include "heuristic-control.h"

void setup() { initSensor(); initMotors(); }

void loop() {
  readSensor(); maximaDisplacement();
  seekAlignment(); runSteppers();
}