#include <Arduino.h>
#include "config.h"

void connectWiFi(); 
void checkWiFi();      
void startSocketTasks(); 
void startBluetooth(); 
void startEthernet();   
extern void startMPU6050Service();
void startBMP180Service();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32...");
  startBMP180Service();
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

