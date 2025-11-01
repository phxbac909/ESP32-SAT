#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

void mpu6050_init();
float mpu6050_roll();
float mpu6050_pitch(); 
float mpu6050_velocity(); // Vận tốc thẳng đứng đã fusion với BMP280
float mpu6050_accel_z(); // Gia tốc Z thô (tuỳ chọn)

#endif