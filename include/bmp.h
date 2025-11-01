#ifndef BMP_H
#define BMP_H

#include <Arduino.h>

bool bmp_init();

float bmp_altitude();

float bmp_temperature();

float bmp_pressure();

#endif