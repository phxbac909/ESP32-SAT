#include <Arduino.h>
#include "mpu6050.h"
#include "bmp.h"
#include "lora_config.h"
#include "gps.h"
#include "config.h"
#include "motor.h"
#include <ESP32_Servo.h>
#include "task_lora.h"
#include "pid_altitude.h"
#include <Wire.h>


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32-SAT...");  

  
  mpu6050_init();
  bmp_init();
    // init_gps();

  motor_init();

  lora_init();

  init_task_receive_data();
  pid_altitude_init();

}

void loop() {

  //   // Read and display filtered temperature
  //   float temperature = bmp_temperature();
  //   Serial.print("Filtered Temperature: ");
  //   Serial.print(temperature);
  //   Serial.println(" °C");
    
  //   Serial.println("--------------------");
    delay(40);
}
