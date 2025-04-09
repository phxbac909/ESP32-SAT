#include "lora.h"
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "lora_config.h"

void loraBegin() {
    Serial.println("Bắt đầu kiểm tra...");

    // Kiểm tra SPI
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    pinMode(LORA_SS, OUTPUT);
    digitalWrite(LORA_SS, HIGH);
    Serial.println("NSS HIGH");
    delay(100);
    digitalWrite(LORA_SS, LOW);
    Serial.println("NSS LOW");
    delay(100);
    digitalWrite(LORA_SS, HIGH);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(0xAA);
    SPI.endTransaction();
    Serial.println("Đã gửi tín hiệu SPI mẫu");

    // Kiểm tra RST
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    Serial.println("RST LOW");
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    Serial.println("RST HIGH");

    // Khởi động LoRa
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(921E6)) {
        Serial.println("Khởi động LoRa thất bại!");
        while (1);
    }
    Serial.println("LoRa khởi động thành công!");
}

void loraSendPacket(int counter) {
    Serial.print("Gửi gói tin: ");
    Serial.println(counter);
    LoRa.beginPacket();
    LoRa.print("Hello LoRa ");
    LoRa.print(counter);
    LoRa.endPacket();
}