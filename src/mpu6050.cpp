#include "mpu6050.h"
#include <MPU6050_tockn.h>

// Biến toàn cục
static MPU6050* mpu = nullptr;
static double sensor_data[3] = {0, 0, 0}; // [roll, pitch, yaw]
static int is_initialized = 0;

void mpu6050_task(void* parameter) {
    while (1) {
        mpu->update();
        sensor_data[0] = mpu->getAngleX();
        sensor_data[1] = mpu->getAngleY(); 
        sensor_data[2] = mpu->getAngleZ();
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz
    }
}

float mpu6050_accel_z(){
    return mpu->getAccZ();
}

void mpu6050_init() {
    if (is_initialized) {
        return ; 
    }    
    Wire.begin(21, 22);
    Wire.setClock(400000); // I2C tốc độ cao    
    mpu = new MPU6050(Wire);
    if (mpu == nullptr) {
        return ; 
    }
    mpu->begin();    
    delay(1000);
    mpu->calcGyroOffsets(true); // Hiệu chỉnh tự động
    xTaskCreate(
        mpu6050_task,
        "mpu6050_task",
        4096,
        NULL,
        1,
        NULL
    );
    is_initialized = 1;
    return ;
}

double* mpu6050_data() { 
    return sensor_data;
}
