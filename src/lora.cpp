
#include <SPI.h>
#include <LoRa.h>
#include "lora_config.h"
#include <vector>
#include <Arduino.h>
#include "bmp.h"
#include "mpu6050.h"
#include "gps.h"
#include "config.h"
#include "motor.h"
#include <ESP32_Servo.h>
#include "pid_altitude.h"

volatile int order = 0;

const int  PIN_LORA_MOSI   =  23 ;
const int  PIN_LORA_MISO   =  19 ;
const int  PIN_LORA_SCK    =  18 ;
const int  PIN_LORA_CS     =  5  ;
const int  PIN_LORA_RST    =  2  ;
const int  PIN_LORA_DIO0   =  4  ;


// Tần số LoRa (921 MHz)
const int LORA_FREQUENCY = 922E6;

std::vector<uint8_t> packet;

void lora_init() {

    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa initialization failed!");
        while (1);
    }

    LoRa.setSpreadingFactor(8);        // SF8 - Cân bằng tốt
    LoRa.setSignalBandwidth(250E3);    // BW250 - Tốc độ cao, đủ nhạy
    LoRa.setCodingRate4(5);           // CR 4/5 - Sửa lỗi vừa phải
    LoRa.setTxPower(17);              // Công suất vừa đủ
    LoRa.setPreambleLength(8);        // Preamble ngắn cho tốc độ
    LoRa.enableCrc();                 // Bật CRC kiểm tra lỗi
    LoRa.setSyncWord(0x12);           // Sync word riêng tránh nhiễu

    Serial.println("LoRa initialized successfully!");
}

// Hàm add_data để thêm số nguyên vào vector
void lora_add_data(int value, size_t num_bytes) {
    // Chuyển giá trị thành byte theo thứ tự big-endian
    for (int i = num_bytes - 1; i >= 0; i--) {
        packet.push_back((value >> (i * 8)) & 0xFF);
    }
}

// Hàm lora_add_data để thêm mảng double
void lora_add_all_data(const double* values, size_t num_values ) {
    for (size_t i = 0; i < num_values; i++) {
    int16_t val = (int16_t)(values[i] * 100);
        lora_add_data(val,2); 
    }
}

void lora_send_data(String data) {
    LoRa.beginPacket();
    LoRa.print(data);
    Serial.print("Sending string : ");
    Serial.println(data);
    packet.clear(); // Xóa dữ liệu trong vector sau khi gửi
    LoRa.flush();
    LoRa.endPacket();
  }

void lora_send_packet() {
    LoRa.beginPacket();
    LoRa.write(packet.data(), packet.size());
    Serial.print("Packet sent (");
    Serial.print(packet.size());
    Serial.print(" bytes): ");

    Serial.println();
    packet.clear(); // Xóa dữ liệu trong vector sau khi gửi
    LoRa.flush(); // Đảm bảo dữ liệu được gửi ngay lập tức
    LoRa.endPacket();
}

String lora_receive_packet() {
   
    int packetSize = LoRa.parsePacket();
    String receivedData = "";
    
    if (packetSize) {
        while (LoRa.available()) {
            receivedData += (char)LoRa.read();
        }
        return receivedData;
    }
    
    return ""; // Trả về string rỗng nếu không có packet
}

char lora_receive_command() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        char command = (char)LoRa.read();
        Serial.print("Received command: ");
        Serial.println(command);
        return command;
    }
    return '\0'; // Trả về ký tự null nếu không có lệnh nào
}

void task_send_data_to_ground_station(void *pvParameters){
  (void)pvParameters; // Bỏ qua tham số nếu không sử dụng

  while (1){

  Serial.println("Send data with order = " + String(order));
  lora_add_data(order,1);
  lora_add_data(0,2);
  lora_add_data(0,2);
  lora_add_data(0,2);
  lora_add_data(0,2);
  lora_add_data(0,2);

//   lora_add_all_data(mpu6050_data(),3);

  TinyGPSLocation location = gps_location();
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

