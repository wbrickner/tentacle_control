#include <AccelStepper.h>
#include "sensor.h"
#include "stepper.h"
#include "heuristic-control.h"

void setup() {
  Serial.begin(115200);
  initSensor(); initMotors();
}

void loop() {
  if (readSensor()) {
    centroidDisplacement(); // read sensor and determine displacement vector
    seekAlignment();        // compute abstract position adjustment
  }

  updateOrientations();
  runSteppers();
}