#include "GPS.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Arduino.h>

// Khởi tạo đối tượng GPS
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

// Hàm khởi tạo GPS
void init_gps() {
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // UART2: GPIO16 (RX), GPIO17 (TX)
}

// Hàm lấy vĩ độ
TinyGPSLocation get_gps_location(){
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }
    if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("No data in gps...");
  }
    return gps.location;
}
