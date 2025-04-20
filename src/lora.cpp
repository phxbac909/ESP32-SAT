
#include <SPI.h>
#include <LoRa.h>
#include "lora_config.h"


const int  PIN_LORA_MOSI   =  23 ;
const int  PIN_LORA_MISO   =  19 ;
const int  PIN_LORA_SCK    =  18 ;
const int  PIN_LORA_CS     =  5  ;
const int  PIN_LORA_RST    =  2  ;
const int  PIN_LORA_DIO0   =  4  ;


// Tần số LoRa (921 MHz)
const int  LORA_FREQUENCY = 922E6;


void lora_begin() {


    LoRa.setPins (PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa initialization failed!");
        while (1);
    }

    LoRa.setSpreadingFactor(7); // Spreading Factor (6-12)
    LoRa.setSignalBandwidth(125E3); // Băng thông tín hiệu (125 kHz)
    LoRa.setCodingRate4(5); // Coding Rate (5-8)

    Serial.println("LoRa initialized successfully!");
}
void lora_send_data(String data) {
    LoRa.beginPacket();
    LoRa.print(data);
    Serial.print("Sending packet: ");
    Serial.println(data);
    LoRa.flush(); // Đảm bảo dữ liệu được gửi ngay lập tức
    LoRa.endPacket();
  }
// Hàm nhận packet
void lora_receive_packet() {
    // Kiểm tra xem có packet nào được nhận không
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // Đọc packet
        Serial.print("Received packet: ");
        while (LoRa.available()) {
            Serial.print((char)LoRa.read());
        }

        // In thông tin RSSI (Received Signal Strength Indicator)
        Serial.print(" with RSSI: ");
        Serial.println(LoRa.packetRssi());
    }
}

