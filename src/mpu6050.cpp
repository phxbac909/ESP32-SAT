#include "mpu6050.h"

// Biến toàn cục
Adafruit_MPU6050 mpu;
float roll = 0, pitch = 0, yaw = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float accXOffset = 0, accYOffset = 0, accZOffset = 0;
unsigned long lastTime = 0;

static bool isActive = true ;
static double data[3];

// Hàm hiệu chỉnh offset
void calibrateMPU6050() {
  const int numSamples = 1000;
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  float accXSum = 0, accYSum = 0, accZSum = 0;

  Serial.println("Đang hiệu chỉnh, giữ cảm biến đứng yên...");
  delay(2000);

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroXSum += g.gyro.x;
    gyroYSum += g.gyro.y;
    gyroZSum += g.gyro.z;
    accXSum += a.acceleration.x;
    accYSum += a.acceleration.y;
    accZSum += a.acceleration.z - 9.81; // Trừ trọng lực
    delay(2);
  }

  gyroXOffset = gyroXSum / numSamples;
  gyroYOffset = gyroYSum / numSamples;
  gyroZOffset = gyroZSum / numSamples;
  accXOffset = accXSum / numSamples;
  accYOffset = accYSum / numSamples;
  accZOffset = accZSum / numSamples;
}

// Hàm khởi tạo MPU6050
void mpu6050_init() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22
  if (!mpu.begin()) {
    Serial.println("Không tìm thấy MPU6050!");
    isActive = false;
    return;
  }
  Serial.println("MPU6050 đã kết nối!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateMPU6050(); // Hiệu chỉnh cảm biến
}

double* mpu6050_data() {
  
  if (!isActive) {
    data[0] = 0;
    data[1] = 0;    
    data[2] = 0;
    Serial.println("MPU6050 không hoạt động!");
    return data;
  }
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Trừ offset
  float accX = a.acceleration.x - accXOffset;
  float accY = a.acceleration.y - accYOffset;
  float accZ = a.acceleration.z - accZOffset;
  float gyroX = g.gyro.x - gyroXOffset;
  float gyroY = g.gyro.y - gyroYOffset;
  float gyroZ = g.gyro.z - gyroZOffset;

  // Tính thời gian chênh lệch
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // Giây
  lastTime = currentTime;

  // Tính Roll và Pitch từ gia tốc kế
  float accRoll = atan2(accY, accZ) * 180 / PI;
  float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Tích hợp con quay
  roll += gyroX * dt * 180 / PI;
  pitch += gyroY * dt * 180 / PI;
  yaw += gyroZ * dt * 180 / PI;

  // Complementary Filter
  float alpha = 0.9;
  roll = alpha * roll + (1 - alpha) * accRoll;
  pitch = alpha * pitch + (1 - alpha) * accPitch;

  Serial.println("Roll: " + String(roll));
  Serial.println("Pitch: " + String(pitch));
  Serial.println("Yaw: " + String(yaw));
 

  // Tạo chuỗi kết quả
  data[0] = roll ;
  data[1] = pitch ;
  data[2] = yaw ; 
  return data;

}