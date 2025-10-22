#include <Arduino.h>
#include "task_lora.h"
#include "bmp.h"
#include "mpu6050.h"
#include "lora_config.h"
#include "gps.h"
#include "config.h"
#include "motor.h"
#include <ESP32_Servo.h>
#include "pid_altitude.h"

volatile int order = 0;

void task_send_data_to_ground_station(void *pvParameters){
  (void)pvParameters; // Bỏ qua tham số nếu không sử dụng

  while (1){

  Serial.println("Send data with order = " + String(order));
  lora_add_data(order,1);
  lora_add_data(0,2);
  lora_add_data(0,2);

  lora_add_all_data(mpu6050_data(),3);

  TinyGPSLocation location = get_gps_location();
  // Hiển thị thông tin
  if (location.isValid()) {
     lora_add_data( (int) location.lng()*1000000,4 );
     lora_add_data( (int) location.lat()*1000000,4 );
  } else {
     lora_add_data( 0,4 );
     lora_add_data( 0,4 );
  }
 
  
  lora_send_packet();
  
  order++;
  
  Serial.println("----------------------------");
  Serial.println("----------------------------");
  
  vTaskDelay(5000 / portTICK_PERIOD_MS); // Đợi 5 giây 
 }
 
}

double* get_number_from_string(String inputString) {
    double* result = new double[4];
    int currentIndex = 0;
    String temp = "";
    
    for (int i = 0; i < inputString.length(); i++) {
        char c = inputString.charAt(i);
        
        if (c == ' ') {
            if (temp.length() > 0) {
                result[currentIndex] = temp.toDouble();
                currentIndex++;
                temp = "";
            }
        } else {
            temp += c;
        }
    }
    
    // Xử lý số cuối cùng (sau dấu cách cuối cùng)
    if (temp.length() > 0 && currentIndex < 4) {
        result[currentIndex] = temp.toDouble();
    }
    
    return result;
}

void task_receive_data_from_ground_station(void *pvParameters){
  (void)pvParameters; // Bỏ qua tham số nếu không sử dụng
 while (1){
  String command = lora_receive_packet();
  if (command != ""){
    Serial.println(command);
    char type = command.charAt(0);
    switch(type) {
      case 't' : // test ping  
        lora_send_data("r");
        break;
        case 's': //stop
        pid_altitude_stop_task();
        break;
      case 'c' : // control
        // double* data = get_number_from_string(command.substring(1));
        // Serial.println(data[0]);
        // pid_altitude_receive_command(data[0]);
        motor_receive_command(command.substring(1));
        break;
    }
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); // Đợi 10 ms để tránh quá tải CPU
 } 
}


void init_task_send_data(){
    xTaskCreate(task_send_data_to_ground_station, "Task send data", 2048, NULL, 4, NULL);
}
void init_task_receive_data(){
    xTaskCreate(task_receive_data_from_ground_station, "Task receive data", 3072, NULL, 1, NULL);
}
void init_task_pid(){

}
