#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "bmp180.h"

Adafruit_BMP085 bmp;
static bool isActive = true ;

void bmp180_init() {
  if (!bmp.begin()) {
    Serial.println("Không tìm thấy BMP180!");
    isActive = false;
    return;
  }
  Serial.println("BMP180 đã được tìm thấy!");
}

String bmp180_data(){
  if (!isActive) {
    Serial.println("BMP180 không hoạt động!");
    return "0-0-0";
  }
  String data = String(bmp.readAltitude()) + " " +
                String(bmp.readTemperature()) + " " +
                String(bmp.readPressure());
  return data;
}