#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "gps.h"

// Biến static để giữ trạng thái GPS nội bộ
static TinyGPSPlus gps;

void gps_init(HardwareSerial* serial, uint32_t baud) {
    serial->begin(baud, SERIAL_8N1, 16, 17); // RX: GPIO16, TX: GPIO17
    Serial.print("GPS UART initialized at baud ");
    Serial.println(baud);
    delay(100);
    if (serial->available()) {
        Serial.println("GPS module is sending data.");
    } else {
        Serial.println("Warning: No data from GPS module. Check connections or power.");
    }
}

bool gps_read_data(HardwareSerial* serial) {
    bool newData = false;
    while (serial->available()) {
        if (gps.encode(serial->read())) {
            newData = true; // Dữ liệu GPS hợp lệ
        }
    }
    return newData;
}

void gps_print_data() {
    if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
        Serial.print("Altitude: ");
        Serial.println(gps.altitude.meters());
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
    } else {
        Serial.println("Waiting for GPS fix...");
    }
}

float gps_get_latitude() {
    return gps.location.isValid() ? gps.location.lat() : 0.0;
}

float gps_get_longitude() {
    return gps.location.isValid() ? gps.location.lng() : 0.0;
}

float gps_get_altitude() {
    return gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
}

uint32_t gps_get_satellites() {
    return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

String gps_get_data_string() {
    String data = "";
    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
        // Vĩ độ
        data += String(gps.location.lat(), 6);
        data += " ";
        // Kinh độ
        data += String(gps.location.lng(), 6);
        data += " ";
        // Độ cao
        data += String(gps.altitude.meters(), 1);   
    } else {
        data = "0.0 0.0 0.0"; // Giá trị mặc định nếu không có tín hiệu
    }
    return data;
}