#include <Arduino.h>
#include "mpu6050.h"
#include "motor.h"
#include <ESP32_Servo.h>
#include "pid_euler.h"
#include <Wire.h>
#include <esp32_now.h>
#include "data_struct.h"
#include "config.h"
#include "pid_euler_test.h"



void setup() {
  SERIAL_BEGIN(115200);

  delay(1000);
  DEBUG_PRINTLN("Starting ESP32-SAT...");  

  mpu6050_init();
  motor_init();
  esp32_now_init();
  pid_euler_init();

}

void loop() {

}



