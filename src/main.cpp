#include <Arduino.h>
#include "wifi.h"
#include "socket.h"
#include "bluetooth.h"
#include "ethernet.h"
#include "mpu6050.h"
#include "bmp180.h"

// void connectWiFi(); 
// void checkWiFi();      
// void startSocketTasks(); 
// void startBluetooth(); 
// void startEthernet();   
// extern void startMPU6050Service();
// void startBMP180Service();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32...");
  startBMP180();
  // startMPU6050Service();
  // connectWiFi();
  // startSocketTasks();
 // startBluetooth();
//  startEthernet();

}

void loop() {
 // checkWiFi();
 // delay(5000);
}

