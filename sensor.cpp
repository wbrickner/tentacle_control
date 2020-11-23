#include "sensor.h"

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

/// safely initialize the sensor
void initSensor() {
  // attempt to initialize the sensor
  while (1) {
    bool success = amg.begin();
    if (success) { break; }
    delay(50);
  }

  // zero-initialize the pixel buffer
  memset(pixels, 0.0, AMG88xx_PIXEL_ARRAY_SIZE * sizeof(float));

  // delay 0.1s for the sensor chip to initialize
  delay(100);
}

/// Read pixel data into the global buffer
void readSensor() { amg.readPixels(pixels); }