#include <Arduino.h>
#include "mpu6050.h"
#include "bmp.h"
#include "lora_config.h"
#include "gps.h"
#include "config.h"
#include "motor.h"
#include <ESP32_Servo.h>
#include "pid_euler.h"
#include <Wire.h>


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32-SAT...");  

  bmp_init(); 
  mpu6050_init();

  // gps_init();

  motor_init();

  lora_init();

  init_task_receive_data();
  // init_task_send_data();
  pid_euler_init();

}

void loop() {

}
