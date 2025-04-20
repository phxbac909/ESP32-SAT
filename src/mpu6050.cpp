#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "mpu6050.h"

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
bool isActive = true;

void setupMpu6050() {
  Wire.begin();
  if (!mpu.begin(0x68)) {
    isActive = false;
    Serial.println("Không tìm thấy MPU6050!");
  }
  Serial.println("MPU6050 đã được tìm thấy!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}

String getMpu6050Data() {
  if (!isActive) {
    Serial.println("MPU6050 không hoạt động!");
    return "0-0-0-0-0-0-0";
  }
  mpu.getEvent(&a, &g, &temp);
  String data = String(a.acceleration.x) + "-" +
                String(a.acceleration.y) + "-" +
                String(a.acceleration.z) + "-" +
                String(g.gyro.x) + "-" +
                String(g.gyro.y) + "-" +
                String(g.gyro.z) + "-" +
                String(temp.temperature);
  return data;
}

