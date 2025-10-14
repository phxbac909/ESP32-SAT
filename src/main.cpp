#include <Arduino.h>
#include "mpu6050.h"
#include "bmp180.h"
#include "lora_config.h"
#include "gps.h"
#include "config.h"

volatile int order = 0;
volatile int number_of_receive = 0;
int time_out;


void task_send_data_to_ground_station(void *pvParameters){
  (void)pvParameters; // Bỏ qua tham số nếu không sử dụng

  while (1){

  Serial.println("Send data with order = " + String(order));
  lora_add_data(order,1);
  lora_add_all_data(bmp180_data(),3); 
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


void task_receive_data_from_ground_station(void *pvParameters){
  (void)pvParameters; // Bỏ qua tham số nếu không sử dụng
 while (1){
  String command = lora_receive_packet();
  if (command != ""){
    if (command== "TP") {
      lora_send_data("R");
      number_of_receive++;
      Serial.println(number_of_receive);
    }
  }
 
  vTaskDelay(10 / portTICK_PERIOD_MS); // Đợi 100 ms để tránh quá tải CPU
 }
 
}



void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32-SAT...");
  
  lora_init();

  // bmp180_init();
  // mpu6050_init();
  // init_gps();
  // xTaskCreate(task_send_data_to_ground_station, "Task send data", 2048, NULL, 4, NULL);
  xTaskCreate(task_receive_data_from_ground_station, "Task receive data", 3072, NULL, 1, NULL);


}

void loop() {

}


