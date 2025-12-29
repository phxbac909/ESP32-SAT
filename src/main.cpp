#include <Arduino.h>
#include "mpu6050.h"
#include "motor.h"
#include <ESP32_Servo.h>
#include "pid_euler.h"
#include <Wire.h>
#include <esp32_now.h>
#include "data_struct.h"
#include "config.h"



void setup() {
  SERIAL_BEGIN(115200);
  delay(1000);
  DEBUG_PRINTLN("Starting ESP32-SAT...");  

   mpu6050_init();
   pid_euler_init();
   esp32_now_init();



}

void loop() {

}
