#include <Arduino.h>
#include "wifi_config.h"
#include "socket.h"
#include "bluetooth.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "lora_config.h"
#include "gps.h"

HardwareSerial SerialGPS(2);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32-SAT...");
  // lora_begin();
  // gps_init(&SerialGPS, 9600); 
  mpu6050_init();
  // bmp180_init();
}

void loop() {
  // lora_send_data("Hello from sattelite!");
  // delay(5000);
//   if (gps_read_data(&SerialGPS)) {
//     String gpsData = gps_get_data_string();
//     Serial.println("GPS Data: " + gpsData);
//  }
//  else {
//     Serial.println("Waiting for GPS fix...");
//  }
// Serial.println(bmp180_data());
// Serial.println(mpu6050_data());
// delay(2000);
}

