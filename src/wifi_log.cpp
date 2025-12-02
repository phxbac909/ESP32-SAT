#include "wifi_log.h"
#include <Arduino.h>

// Khởi tạo biến toàn cục
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

void wifi_init() {
    Serial.println("Đang khởi tạo Access Point...");
    
    WiFi.mode(WIFI_AP);
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    WiFi.softAPConfig(local_IP, gateway, subnet);
    
    if (WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("Access Point khởi tạo thành công!");
        Serial.print("SSID: ");
        Serial.println(WIFI_SSID);
        Serial.print("IP Address: ");
        Serial.println(WiFi.softAPIP());
        
        // Khởi động TCP server
        tcpServer.begin();
        Serial.println("TCP Server đã khởi động trên port 8888");
        Serial.println("Đang chờ client kết nối...");
        
        // Chờ client kết nối ngay khi khởi tạo
        while (!tcpClient || !tcpClient.connected()) {
            tcpClient = tcpServer.available();
            if (tcpClient) {
                Serial.println("Client đã kết nối thành công!");
                break;
            }
            delay(500);
            Serial.print(".");
        }
    } else {
        Serial.println("Lỗi khởi tạo Access Point!");
    }
}

void wifi_logf(const char* format, ...) {
    // Kiểm tra và chấp nhận kết nối mới nếu client đã ngắt kết nối
    if (!tcpClient || !tcpClient.connected()) {
        tcpClient = tcpServer.available();
        if (!tcpClient) {
            return; // Không có client kết nối
        }
    }
    
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (tcpClient.connected()) {
        tcpClient.println(buffer);
        tcpClient.flush();
        Serial.print("[Đã gửi] ");
        Serial.println(buffer);
    }
}