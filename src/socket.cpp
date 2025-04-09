#include <Arduino.h>
#include <WiFi.h>
#include "socket.h"

const int SERVER_PORT = 8080;
const char* TARGET_IP = "192.168..109";
const int TARGET_PORT = 8081;

WiFiServer server(SERVER_PORT);
WiFiClient sendClient;
TaskHandle_t serverTaskHandle = NULL;
TaskHandle_t sendTaskHandle = NULL;

void serverTask(void *parameter) {
  while (true) {
    WiFiClient client = server.available();
    if (client) {
      Serial.println("New client connected");
      while (client.connected()) {
        if (client.available()) {
          String receivedText = client.readStringUntil('\n');
          Serial.print("Received: ");
          Serial.println(receivedText);
          client.println("ESP32 received: " + receivedText);
        }
      }
      client.stop();
      Serial.println("Client disconnected");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void sendMessage(char * message) {

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!sendClient.connected()) { 
        if (sendClient.connect(TARGET_IP, TARGET_PORT)) {
          Serial.println("Connected to target server");
        } else {
          Serial.println("Failed to connect to target");
          vTaskDelay(5000 / portTICK_PERIOD_MS);
          continue; // Thử lại sau 5 giây
        }
      }

      if (sendClient.connected()) {
        sendClient.println("0-1-2-3-4-5-6-7-8-9-10-11"); // Gửi dữ liệu
        Serial.println("Sent !");
      } else {
        Serial.println("Lost connection to server");
        sendClient.stop(); // Đóng kết nối để thử lại
      }
    } else {
      Serial.println("WiFi not connected");
      sendClient.stop(); // Đóng kết nối nếu WiFi mất
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Đợi 5 giây
  }

}

void startSocketServer() {
  server.begin();
  Serial.println("Socket server started on port 8080");

  xTaskCreate(serverTask, "ServerTask", 4096, NULL, 1, &serverTaskHandle);
}