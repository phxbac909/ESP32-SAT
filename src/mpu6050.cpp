#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "mpu6050.h"

Adafruit_MPU6050 mpu;

void mpu6050Service(void *pvParameters) {
  Wire.begin(); // I2C mặc định: SDA = D21, SCL = D22
  if (!mpu.begin(0x68)) {
    Serial.println("Không tìm thấy MPU6050!");
    while (1) vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  Serial.println("MPU6050 đã được tìm thấy!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  while (1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.println("=== MPU6050 ===");
    Serial.print("Gia tốc X: "); Serial.print(a.acceleration.x); Serial.println(" m/s^2");
    Serial.print("Gia tốc Y: "); Serial.print(a.acceleration.y); Serial.println(" m/s^2");
    Serial.print("Gia tốc Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s^2");
    Serial.print("Góc quay X: "); Serial.print(g.gyro.x); Serial.println(" rad/s");
    Serial.print("Góc quay Y: "); Serial.print(g.gyro.y); Serial.println(" rad/s");
    Serial.print("Góc quay Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");
    Serial.print("Nhiệt độ MPU: "); Serial.print(temp.temperature); Serial.println(" °C");

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Đợi 2 giây
  }
}

void startMPU6050() {
  xTaskCreate(
    mpu6050Service,   // Hàm service
    "MPU6050Service", // Tên task
    4096,             // Kích thước stack
    NULL,             // Tham số truyền vào
    1,                // Độ ưu tiên
    NULL              // Handle của task
  );
}