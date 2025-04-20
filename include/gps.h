#ifndef GPS_H
#define GPS_H

#include <HardwareSerial.h>
#include <Arduino.h> // Thêm để dùng kiểu String

// Khai báo các hàm
void gps_init(HardwareSerial* serial, uint32_t baud);
bool gps_read_data(HardwareSerial* serial);
void gps_print_data();
float gps_get_latitude();
float gps_get_longitude();
float gps_get_altitude();
uint32_t gps_get_satellites();
String gps_get_data_string(); // Hàm mới trả về chuỗi dữ liệu

#endif