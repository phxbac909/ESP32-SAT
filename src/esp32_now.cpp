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
ControlData data;

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
        // DEBUG_PRINT("Data sent successfully, length: ");
        // DEBUG_PRINTLN(len);
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
            
            // 1. SỬA: Kiểm tra đủ độ dài gói tin (1 + 2 + 4 + 4 + 4 = 15 bytes)
            if (length < 15) {
                DEBUG_PRINTLN("Err: Ctrl Len < 15");
                return;
            }

            // 2. Parse dữ liệu            
            // Đọc Speed (Little Endian)
            data.speed = (int16_t)(incomingData[1] | (incomingData[2] << 8));
            
            // Đọc Floats (memcpy an toàn nhất)
            memcpy(&data.roll,  &incomingData[3], 4);
            memcpy(&data.pitch, &incomingData[7], 4);
            memcpy(&data.yaw,   &incomingData[11], 4);
            
            // 3. Gửi vào bộ điều khiển
            pid_euler_set_base_throttle(data.speed);
            pid_euler_set_angle(data.roll, data.pitch, data.yaw);
            
            // 4. LOG 1 DÒNG NGẮN GỌN (Dùng printf để format đẹp)
      
            // DEBUG_PRINTF("Ctrl S:%d R:%.2f P:%.2f Y:%.2f\n", 
            //             data.speed, data.roll, data.pitch, data.yaw);

            break;
        }
       case 2: { // PidEulerData
            // Kiểm tra độ dài: 1 byte ID + Kích thước thực tế của struct PidData
            if (length < (sizeof(PidData) + 1)) {
                DEBUG_PRINTF("Err: PID Len %d < Required %d\n", length, sizeof(PidData) + 1);
                return;
            }
            
            PidData receivedPid;
            
            // Copy toàn bộ dữ liệu an toàn từ byte thứ 1 (bỏ qua byte 0 là ID) vào struct
            memcpy(&receivedPid, &incomingData[1], sizeof(PidData));
            
            // Cập nhật vào hệ thống PID
            pid_euler_set_tuning(receivedPid);
            
            // Log ngắn gọn để xác nhận trên Serial
            DEBUG_PRINTF("=== PID Parameters Updated via ESP-NOW : %.2f\n",receivedPid.angle_kd_pitch);
            break;
        }
        
      
        case 0: { 
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

