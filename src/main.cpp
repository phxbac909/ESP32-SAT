#include <Arduino.h>
#include "wifi_config.h"
#include "socket.h"
#include "bluetooth.h"
#include "ethernet_config.h"
#include "mpu6050.h"
#include "bmp180.h"



void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32...");
  // startBMP180();
  // startMPU6050Service();
  startWiFi();
  startSocketServer();
  sendMessage("0-0-0-0-0-0-0-0-0-0-0");
  
 // startBluetooth();
//  startEthernet();

}

void loop() {
 // checkWiFi();
 // delay(5000);
}

