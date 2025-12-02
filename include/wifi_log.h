#ifndef WIFI_LOG_H
#define WIFI_LOG_H

#include <WiFi.h>
#include <WiFiClient.h>

// Các hằng số cấu hình
#define WIFI_SSID "ESP32-SAT"
#define WIFI_PASSWORD "adminSAT"
#define SERVER_IP "192.168.4.1"
#define TCP_PORT 8888

// Khai báo biến toàn cục
extern WiFiClient tcpClient;
extern bool isConnected;

// Khai báo hàm
void wifi_init();
void wifi_logf(const char* format, ...);

#endif