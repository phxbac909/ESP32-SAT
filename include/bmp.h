#ifndef BMP_H
#define BMP_H

#include <Arduino.h>

// Initialize BMP280 sensor
bool bmp_init();

// Get filtered altitude in meters
float bmp_altitude();

// Get filtered temperature in Celsius
float bmp_temperature();

#endif