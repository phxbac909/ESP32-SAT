#include "mpu6050.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "config.h"


static MPU6050* mpu = nullptr;
static bool is_initialized = false;
static float current_roll = 0.0f;
static float current_yaw = 0.0f;
static float current_pitch = 0.0f; 

void mpu6050_task(void* parameter) {
    while (1) {
        mpu->update();
        current_roll = mpu->getAngleX();
        current_pitch = mpu->getAngleY();
        current_yaw = mpu->getAngleZ();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void mpu6050_init() {
    if (is_initialized) return;
    
    Wire.begin(21, 22);
    Wire.setClock(400000);
    mpu = new MPU6050(Wire);
    if (mpu == nullptr) return;
    
    mpu->begin();
    delay(1000);

    mpu->calcGyroOffsets(true);

    xTaskCreate(mpu6050_task, "fusion", 4096, NULL, 1, NULL);
    is_initialized = true;
}

float mpu6050_roll() { return current_roll; }
float mpu6050_pitch() { return current_pitch; }
