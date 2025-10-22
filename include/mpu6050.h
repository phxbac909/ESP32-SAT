#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

void mpu6050_init();
double* mpu6050_data();
float mpu6050_accel_z();

#ifdef __cplusplus
}
#endif

#endif