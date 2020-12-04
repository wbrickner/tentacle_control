#include "sensor.h"

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE] = { 0 };
unsigned long lastSampleTime;
// volatile bool imageReady = false;

/// safely initialize the sensor
void initSensor() {
  while (1) {
    if (amg.begin()) { break; } // attempt to initialize the sensor
    Serial.println("Sensor initialization failed, retying in 250ms.");
    delay(250);                 // if the sensor did not initialize, delay 250ms and try again
  }

  // delay 100ms for the sensor chip to "warm up" (ba dum pssk)
  delay(100);

  // initialize the last sample time
  lastSampleTime = 0UL;

  Serial.println("Sensor initialization complete.");
}

/// read pixel data into the global buffer
bool readSensor() {
  unsigned long now = millis();
  if (now - lastSampleTime <= SAMPLE_PERIOD) { return false; }
  
  // read the entire image into our buffer and remember the last sampling time
  lastSampleTime = now;
  amg.readPixels(pixels);
  return true;
}