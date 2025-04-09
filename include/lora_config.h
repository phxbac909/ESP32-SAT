#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <SPI.h>
#include <LoRa.h>

// Định nghĩa các chân kết nối RA-01H với ESP32
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5    // NSS
#define LORA_RST 4
#define LORA_DIO0 2

void loraBegin();           // Khởi tạo LoRa
void loraSendPacket(int counter); // Gửi gói tin LoRa

#endif