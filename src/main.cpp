#include <Arduino.h>
#include "wifi_config.h"
#include "socket.h"
#include "bluetooth.h"
#include "ethernet_config.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "lora_config.h"


int counter = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32...");
  loraBegin();
  // startBMP180();
  // startMPU6050();
  // startWiFi();
  // startSocketServer();
  // sendMessage("0-0-0-0-0-0-0-0-0-0-0");
  
 // startBluetooth();
//  startEthernet();

}

void loop() {
  loraSendPacket(counter);
  counter++;
  delay(5000); 
 // checkWiFi();
}

