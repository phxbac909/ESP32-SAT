#include <Arduino.h>
#include <WiFi.h>
#include "wifi.h"

const char* SSID = "Disconnect";
const char* PASSWORD = "254/101/17/24";

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