#include "mpu6050.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>



// --- BIẾN TOÀN CỤC ---
static MPU6050* mpu = nullptr;
static bool is_initialized = false;
static imu_data_t g_imu_data = {0}; 

// Biến lưu thời gian để tính dt
static unsigned long last_time_us = 0; 

// --- HẰNG SỐ TOÁN HỌC ---
#define DEG_TO_RAD 0.01745329f  // Hệ số đổi Độ sang Radian (pi/180)

void mpu6050_init() {
    if (is_initialized) return;
    
    Wire.begin(21, 22);
    Wire.setClock(400000); // I2C Fast Mode
    mpu = new MPU6050(Wire);
    if (mpu == nullptr) return;
    
    mpu->begin();
    
    // --- BẬT LỌC PHẦN CỨNG (DLPF) ---
    Wire.beginTransmission(0x68);
    Wire.write(0x1A); 
    Wire.write(0x03); 
    Wire.endTransmission();
    
    delay(1000);
    mpu->calcGyroOffsets(true);

    // Lưu mốc thời gian ban đầu
    last_time_us = micros(); 
    is_initialized = true;
}

// Hàm này sẽ được gọi liên tục bên trong Task PID của bạn
void IMU_Update_And_Read(imu_data_t *out_data) {
    if (!is_initialized || out_data == nullptr) return;

    // Cập nhật dữ liệu từ cảm biến
    mpu->update();

    // --- TÍNH TOÁN dt (dựa trên micros() để có độ chính xác cao) ---
    unsigned long current_time_us = micros();
    float dt = (current_time_us - last_time_us) / 1000000.0f; // Đổi ra giây (s)
    last_time_us = current_time_us;

    // Kẹp dt (Safeguard): Nếu vòng lặp bị kẹt quá lâu (> 50ms), giới hạn dt để không làm nổ bộ lọc
    if (dt > 0.05f) dt = 0.004f; 

    // 1. Lọc thông thấp (Low Pass Filter) 0.7 - 0.3 cho Gyro
    g_imu_data.gyro_roll  = (g_imu_data.gyro_roll * 0.7f)  + (mpu->getGyroX() * 0.3f);
    g_imu_data.gyro_pitch = (g_imu_data.gyro_pitch * 0.7f) + (mpu->getGyroY() * 0.3f);
    g_imu_data.gyro_yaw   = (g_imu_data.gyro_yaw * 0.7f)   + (mpu->getGyroZ() * 0.3f);

    // Lấy góc thô từ Accelerometer
    g_imu_data.acc_roll  = mpu->getAccAngleX();
    g_imu_data.acc_pitch = mpu->getAccAngleY();

    // 2. Tích phân Gyro (Vận tốc góc * dt)
    g_imu_data.roll  += g_imu_data.gyro_roll * dt;
    g_imu_data.pitch += g_imu_data.gyro_pitch * dt;
    g_imu_data.yaw   += g_imu_data.gyro_yaw * dt;

    // 3. Quy đổi Roll/Pitch khi xoay Yaw (Chuyển giao hệ quy chiếu)
    float yaw_rad_step = g_imu_data.gyro_yaw * dt * DEG_TO_RAD; 
    
    float roll_temp = g_imu_data.roll; 
    g_imu_data.roll  += g_imu_data.pitch * sin(yaw_rad_step);
    g_imu_data.pitch -= roll_temp * sin(yaw_rad_step);

    // 4. Bộ lọc bù (Complementary Filter) 99.96% - 0.04% chống trôi
    g_imu_data.roll  = g_imu_data.roll * 0.9996f + g_imu_data.acc_roll * 0.0004f;
    g_imu_data.pitch = g_imu_data.pitch * 0.9996f + g_imu_data.acc_pitch * 0.0004f;
    
    // Đẩy dữ liệu ra con trỏ struct của hàm gọi
    *out_data = g_imu_data;
}