#ifndef MPU6050_H
#define MPU6050_H
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


void mpu6050_init();
double* mpu6050_data();

#endif