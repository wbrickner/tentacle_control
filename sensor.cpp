#include "sensor.h"

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE] = { 0 };
volatile bool imageReady = false;

/// invoked by hardware interrupt when new image data is available
void onImageData() { imageReady = true; }

void registerInterrupt() {
  amg.setInterruptMode(AMG88xx_DIFFERENCE);
  amg.enableInterrupt();
  pinMode(IR_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_INT_PIN), onImageData, FALLING);
}

/// safely initialize the sensor
void initSensor() {
  while (1) {
    if (amg.begin()) { break; } // attempt to initialize the sensor
    delay(50);                  // if the sensor did not initialize, delay 50ms and try again
  }

  // attach hardware interrupt for when image data is available
  registerInterrupt();

  // delay 100ms for the sensor chip to "warm up" (ba dum pssk)
  delay(100);
}

/// read pixel data into the global buffer
void readSensor() {
  amg.readPixels(pixels); // read the entire image into our buffer
  amg.clearInterrupt();   // clear the interrupt state so we may receive the next interrupt
  imageReady = false;     // reset the imageReady flag
}