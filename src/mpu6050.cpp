#include "mpu6050.h"
#include "bmp.h"
#include <MPU6050_tockn.h>
#include <Wire.h>

static MPU6050* mpu = nullptr;
static bool is_initialized = false;
static float current_roll = 0.0f;
static float current_pitch = 0.0f; 
static float current_velocity = 0.0f; // m/s
static float previous_altitude = 0.0f;
static unsigned long last_fusion_time = 0;
static float initial_gravity = 0.0f;
float pitch = 0.0f, roll = 0.0f;

void mpu6050_task(void* parameter) {
    
    previous_altitude = bmp_altitude();
    last_fusion_time = micros();
    
    while (1) {
        unsigned long current_time = micros();
        float dt = (current_time - last_fusion_time) / 1000000.0f;
        if (dt > 0.02f) dt = 0.01f;
        if (dt <= 0.0f) dt = 0.01f;
        
        mpu->update();
        current_roll = mpu->getAngleX();
        current_pitch = mpu->getAngleY();
        
        // // === GIA TỐC THẲNG ĐỨNG ĐƠN GIẢN ===
        float raw_accel_z = mpu->getAccZ();
        float vertical_accel = 
            mpu->getAccX() * -sin(radians(mpu->getAngleX())) -
            mpu->getAccY() * sin(radians(mpu->getAngleY())) * cos(radians(mpu->getAngleX())) +
            mpu->getAccZ() * cos(radians(mpu->getAngleY())) * cos(radians(mpu->getAngleX())) -
            0.87f;  

        float current_altitude = bmp_altitude();
        float baro_velocity = (current_altitude - previous_altitude) / dt; // m/s
        
        // === COMPLEMENTARY FILTER 98-2 ===
        current_velocity += vertical_accel * dt; // Tích phân gia tốc
        current_velocity = 0.98f * current_velocity + 0.02f * baro_velocity; // Fusion
        
//         // Debug
//         static int count = 0;
//         if (count++ % 20 == 0) {
//             Serial.printf("V:%.3f m/s | Acc:%.3f | Baro:%.3f | Roll:%.2f | Pitch : %.2f ", 
//                          current_velocity, vertical_accel, baro_velocity,current_roll, current_pitch);
//                          Serial.print("AccX:"); Serial.print(mpu->getAccX());
// Serial.print(" AccY:"); Serial.print(mpu->getAccY());
// Serial.print(" AccZ:"); Serial.println(mpu->getAccZ());
//         }

        
        previous_altitude = current_altitude;
        last_fusion_time = current_time;
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
    // Gravity calibration
    Serial.println();
    Serial.println("Calibrating gravity...");
    float sum_accel_z = 0.0f;
    for (int i = 0; i < 30; i++) {
        mpu->update();
        sum_accel_z += mpu->getAccZ();
        delay(30);
    }
    initial_gravity = sum_accel_z / 30.0f;
    Serial.printf("Gravity: %.3f g\n", initial_gravity);
    xTaskCreate(mpu6050_task, "fusion", 4096, NULL, 1, NULL);
    is_initialized = true;
}

float mpu6050_roll() { return current_roll; }
float mpu6050_pitch() { return current_pitch; }
float mpu6050_velocity() { return current_velocity; } // m/s
float mpu6050_accel_z() { return (mpu->getAccZ() - initial_gravity) * 9.81f; } // m/s²
void mpu6050_reset_velocity() { current_velocity = 0.0f; }