#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "data_struct.h"
#include "pid_euler.h"
#include "config.h"
#include "motor.h"


// Biến toàn cục
esp_now_peer_info_t peerInfo;
uint8_t broadcastAddress[] = {0x14, 0x33, 0x5C, 0x2F, 0x1F, 0xB4}; // Broadcast address
bool data_received = false;
uint8_t received_data[250]; // Buffer nhận dữ liệu
int received_data_len = 0;

// Khai báo prototype hàm callback
void on_data_receive(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

// Hàm khởi tạo ESP-NOW
void esp32_now_init() {
    // Set device as WiFi Station
    WiFi.mode(WIFI_STA);
    
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        DEBUG_PRINTLN("Error initializing ESP-NOW");
        return;
    }
    
    // Register callback functions
    esp_now_register_recv_cb(on_data_receive);
    
    // Register peer (broadcast)
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        DEBUG_PRINTLN("Failed to add peer");
        return;
    }
    
    DEBUG_PRINTLN("ESP-NOW Initialized Successfully");
}

// Hàm gửi dữ liệu qua ESP-NOW
bool esp32_now_send(const uint8_t *data, int len) {
    // Kiểm tra độ dài dữ liệu
    if (len > 250) {
        DEBUG_PRINTLN("Data too long (max 250 bytes)");
        return false;
    }
    
    // Gửi dữ liệu
    esp_err_t result = esp_now_send(broadcastAddress, data, len);
    
    if (result == ESP_OK) {
        DEBUG_PRINT("Data sent successfully, length: ");
        DEBUG_PRINTLN(len);
        return true;
    } else {
        DEBUG_PRINT("Error sending data: ");
        DEBUG_PRINTLN(result);
        return false;
    }
}


void proceedReceivedData(uint8_t *incomingData, size_t length) {
    if (length < 1 || incomingData == nullptr) {
        DEBUG_PRINTLN("Error: Invalid data");
        return;
    }
    
    uint8_t id = incomingData[0];
    
    switch (id) {
        case 1: { // ControlData
            if (length < 3) {
                DEBUG_PRINTLN("Error: Invalid ControlData length");
                return;
            }
            
            ControlData controlData;
            controlData.id = incomingData[0];
            // Little Endian: byte thấp trước
            controlData.speed = (int16_t)(incomingData[1] | (incomingData[2] << 8));
            
            DEBUG_PRINTLN("=== Control Data ===");
            DEBUG_PRINT("ID: ");
            DEBUG_PRINTLN(controlData.id);
            DEBUG_PRINT("Speed: ");
            DEBUG_PRINTLN(controlData.speed);
            DEBUG_PRINTLN("===================");
            
            pid_euler_set_base_throttle(controlData.speed);
            
            break;
        }
        
        case 2: { // PidEulerData
            if (length < 13) {
                DEBUG_PRINTLN("Error: Invalid PidEulerData length");
                return;
            }
            
            PidEulerData pidData;
            pidData.id = incomingData[0];
            
            // Đọc float theo Little Endian (4 bytes mỗi float)
            memcpy(&pidData.kp, &incomingData[1], 4);
            memcpy(&pidData.ki, &incomingData[5], 4);
            memcpy(&pidData.kd, &incomingData[9], 4);
         
            pid_euler_set_roll_pid(pidData.kp,pidData.ki,pidData.kd);
            
            break;
        }
        
         case 3: { // PidEulerData
            if (length < 13) {
                DEBUG_PRINTLN("Error: Invalid PidEulerData length");
                return;
            }
            
            PidEulerData pidData;
            pidData.id = incomingData[0];
            
            // Đọc float theo Little Endian (4 bytes mỗi float)
            memcpy(&pidData.kp, &incomingData[1], 4);
            memcpy(&pidData.ki, &incomingData[5], 4);
            memcpy(&pidData.kd, &incomingData[9], 4);
            
            pid_euler_set_pitch_pid(pidData.kp,pidData.ki,pidData.kd);

   
            
            break;
        }
        
        case 0: { // StopSignal
            if (length < 1) {
                DEBUG_PRINTLN("Error: Invalid StopSignal length");
                return;
            }
            
            StopSignal stopSignal;
            stopSignal.id = incomingData[0];
            
            DEBUG_PRINTLN("=== Stop Signal ===");
            DEBUG_PRINT("ID: ");
            DEBUG_PRINTLN(stopSignal.id);
            DEBUG_PRINTLN("STOP Command Received!");
            DEBUG_PRINTLN("==================");
            
            motor_detach();
            break;
        }
        
        default:
            DEBUG_PRINT("Error: Unknown packet ID: ");
            DEBUG_PRINTLN(id);
            break;
    }
}
// Callback khi nhận dữ liệu (đã đăng ký trong init)
void on_data_receive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    // Sao chép dữ liệu vào buffer
    if (len <= 250) {
       proceedReceivedData((uint8_t*) incomingData,len);
    }
}

