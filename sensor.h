#pragma once

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

#define IR_SCL_PIN 0
#define IR_SDA_PIN 0
#define IR_INT_PIN 2

#define SAMPLE_PERIOD 100 // minimum time between IR samples [ms]
#define SENSOR_COLUMNS 8
#define SENSOR_SIZE AMG88xx_PIXEL_ARRAY_SIZE

extern float pixels[SENSOR_SIZE];

void initSensor();
bool readSensor();