#include <Arduino.h>
#include <WiFi.h>
#include "socket.h"

const int SERVER_PORT = 8080;
const char* TARGET_IP = "192.168.1.109";
const int TARGET_PORT = 8081;

WiFiServer server(SERVER_PORT);
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

void sendTask(void *parameter) {
  WiFiClient sendClient;
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      if (sendClient.connect(TARGET_IP, TARGET_PORT)) {
        sendClient.println("ESP32 alive");
        Serial.println("Sent: ESP32 alive");
        sendClient.stop();
      } else {
        Serial.println("Failed to connect to target");
      }
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void startSocketTasks() {
  server.begin();
  Serial.println("Socket server started on port 8080");

  xTaskCreate(serverTask, "ServerTask", 4096, NULL, 1, &serverTaskHandle);
  xTaskCreate(sendTask, "SendTask", 4096, NULL, 1, &sendTaskHandle);
}