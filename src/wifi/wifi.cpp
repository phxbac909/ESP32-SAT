#include <Arduino.h>
#include <WiFi.h>
#include "config.h"

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

void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Disconnected, reconnecting...");
    WiFi.reconnect();
  }
}