#ifndef GPS_H
#define GPS_H

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Arduino.h>

// Khai báo đối tượng GPS
extern TinyGPSPlus gps;
extern HardwareSerial SerialGPS;

// Hàm khởi tạo GPS
void init_gps();

TinyGPSLocation get_gps_location();

#endif