#include "mpu6050.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "config.h"



static MPU6050* mpu = nullptr;
static bool is_initialized = false;

// Biến lưu Gyro
static float gyro_roll_input = 0.0f;
static float gyro_pitch_input = 0.0f;
static float gyro_yaw_input = 0.0f;

// Biến lưu Angle
static float current_roll = 0.0f;
static float current_pitch = 0.0f;
static float current_yaw = 0.0f;




float mpu6050_gyro_roll() { return gyro_roll_input; }
float mpu6050_gyro_pitch() { return gyro_pitch_input; }
float mpu6050_gyro_yaw() { return gyro_yaw_input; }
float mpu6050_roll() { return current_roll; }
float mpu6050_pitch() { return current_pitch; }
float mpu6050_yaw() { return current_yaw; }
void mpu6050_task(void* parameter) {
    while (1) {
        mpu->update();
        
        // 1. Lấy góc (Angle) - Dùng cho PID vòng ngoài
        current_roll = mpu->getAngleX();
        current_pitch = mpu->getAngleY();
        current_yaw = mpu->getAngleZ();
        
        // 2. Lấy Gyro thô và Lọc qua Kalman
         gyro_roll_input = mpu->getGyroX();
         gyro_pitch_input = mpu->getGyroY();
         gyro_yaw_input = mpu->getGyroZ();
        
        // Tốc độ loop đọc cảm biến (khoảng 200Hz-250Hz là đẹp)
        vTaskDelay(4 / portTICK_PERIOD_MS); 
    }
}

void mpu6050_init() {
    if (is_initialized) return;
    
    Wire.begin(21, 22);
    Wire.setClock(400000); // I2C Fast Mode
    mpu = new MPU6050(Wire);
    if (mpu == nullptr) return;
    
    mpu->begin();
    
    // --- BẬT LỌC PHẦN CỨNG (DLPF) ---
    // Kalman hoạt động tốt nhất khi tín hiệu đã được lọc sơ bằng phần cứng
    // Config 0x03 (~42Hz bandwidth) loại bỏ nhiễu rung cơ khí tần số cao
    Wire.beginTransmission(0x68);
    Wire.write(0x1A); 
    Wire.write(0x03); 
    Wire.endTransmission();
    
    delay(1000);
    mpu->calcGyroOffsets(true);

    xTaskCreate(mpu6050_task, "imu", 4096, NULL, 2, NULL);
    is_initialized = true;
}

