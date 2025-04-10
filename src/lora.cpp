// include/lora_config.h
#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <SPI.h>
#include <LoRa.h>

// Định nghĩa các chân cho LoRa RA-01H
#define LORA_SS   5U
#define LORA_RST  2U
#define LORA_DIO0 4U

// Định nghĩa các chân SPI
#define SPI_SCK   18U
#define SPI_MOSI  23U
#define SPI_MISO  19U

#define PIN_LORA_COPI   23
#define PIN_LORA_CIPO   19
#define PIN_LORA_SCK    18
#define PIN_LORA_CS     5
#define PIN_LORA_RST    2
#define PIN_LORA_DIO0   4


// Tần số LoRa (921 MHz)
#define LORA_FREQUENCY 921E6

// Hàm khởi tạo LoRa
void lora_begin() {
    // Khởi tạo SPI với các chân đã định nghĩa
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, LORA_SS);

    // Khởi tạo module LoRa
    // LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    LoRa.setPins (PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

    // Bắt đầu LoRa với tần số 921 MHz
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa initialization failed!");
        while (1); // Dừng chương trình nếu khởi tạo thất bại
    }

    // Bỏ qua cấu hình các tham số vì chỉ nhận packet
    // Nếu cần, bạn có thể thêm lại các tham số sau để khớp với bên gửi:
    // LoRa.setSpreadingFactor(7); // Spreading Factor (6-12)
    // LoRa.setSignalBandwidth(125E3); // Băng thông tín hiệu (125 kHz)
    // LoRa.setCodingRate4(5); // Coding Rate (5-8)

    Serial.println("LoRa initialized successfully!");
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

#endif 