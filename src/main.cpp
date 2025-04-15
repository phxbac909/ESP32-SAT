#include <Arduino.h>
#include "wifi_config.h"
#include "socket.h"
#include "bluetooth.h"
#include "ethernet_config.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "lora_config.h"


int counter = 0;

void lora_begin();
void lora_receive_packet();
void lora_send_data(String data);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32-SAT...");
  lora_begin();
}

void loop() {
  lora_send_data("Hello from sattelite!");
  delay(5000);
  // loraSendPacket(counter);
  // counter++;
 // checkWiFi();
}

