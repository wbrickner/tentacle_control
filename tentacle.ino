#include <AccelStepper.h>
#include "sensor.h"
#include "stepper.h"
#include "heuristic-control.h"

void setup() { initSensor(); initMotors(); }

void loop() {
  if (imageReady) {
    readSensor(); maximaDisplacement(); // read sensor and determine displacement vector
    seekAlignment();                    // compute abstract position adjustment
  }
  
  runSteppers();
}