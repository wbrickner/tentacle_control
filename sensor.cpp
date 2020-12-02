#include "sensor.h"

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE] = { 0 };
volatile bool imageReady = false;

/// invoked by hardware interrupt when new image data is available
void onImageData() { imageReady = true; }

/// attach hardware interrupt for when image data is available
void registerInterrupt() {
  // prepare the interrupt pin
  pinMode(IR_INT_PIN, INPUT);

  // register interrupts with the AMG library
  amg.setInterruptMode(AMG88xx_DIFFERENCE); amg.enableInterrupt();

  // actually register hardware interrupt
  attachInterrupt(digitalPinToInterrupt(IR_INT_PIN), onImageData, FALLING);
}

/// safely initialize the sensor
void initSensor() {
  while (1) {
    if (amg.begin()) { break; } // attempt to initialize the sensor
    delay(250);                 // if the sensor did not initialize, delay 250ms and try again
  }

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