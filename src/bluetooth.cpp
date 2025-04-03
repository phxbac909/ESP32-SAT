#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "bluetooth.h"

const char* BT_NAME = "ESP32_BT";

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
TaskHandle_t bluetoothTaskHandle = NULL;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Callback để xử lý khi client ghi dữ liệu
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Received via BLE: ");
      Serial.println(value.c_str());
      // Phản hồi lại client (tùy chọn)
      pCharacteristic->setValue("ESP32 received: " + value);
      pCharacteristic->notify();
    }
  }
};

// Task gửi dữ liệu định kỳ
void bluetoothTask(void *parameter) {
  while (true) {
    pCharacteristic->setValue("ESP32 BLE alive");
    pCharacteristic->notify();
    Serial.println("Sent via BLE: ESP32 BLE alive");
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Gửi mỗi 5 giây
  }
}

void startBluetooth() {
  BLEDevice::init(BT_NAME);
  pServer = BLEDevice::createServer();
  BLEService* pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  
  pCharacteristic->addDescriptor(new BLE2902()); // Cần cho notify
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); // Thêm callback
  pCharacteristic->setValue("Hello BLE");
  
  pService->start();
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("BLE started: " + String(BT_NAME));

  xTaskCreate(bluetoothTask, "BluetoothTask", 4096, NULL, 1, &bluetoothTaskHandle);
}