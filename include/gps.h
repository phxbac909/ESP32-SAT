#ifndef GPS_H
#define GPS_H

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Arduino.h>

// Khai báo đối tượng GPS
extern TinyGPSPlus gps;
extern HardwareSerial SerialGPS;

void gps_init() ;

// Hàm lấy vĩ độ
TinyGPSLocation gps_location();

#endif