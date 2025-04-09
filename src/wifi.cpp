#include <Arduino.h>
#include <WiFi.h>
#include "wifi_config.h"

const char* SSID = "Disconnect";
const char* PASSWORD = "254/101/17/24";

TaskHandle_t wifiTaskHandle = NULL;

void connectWiFi() {
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to ");
  Serial.print(SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void checkWiFi(void *parameter) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Disconnected, reconnecting...");
    WiFi.reconnect();
  }
}

void startWiFi() {
    connectWiFi();
    // xTaskCreate(
    //   checkWiFi,       // Hàm service
    //   "WiFiService",   // Tên task
    //   2048 ,            // Kích thước stack
    //   NULL,            // Tham số truyền vào
    //   1,               // Độ ưu tiên
    //   &wifiTaskHandle   // Handle của task
    // );
}
