#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "bmp180.h"

Adafruit_BMP085 bmp;
static bool isActive = true ;

void bmp180_init() {
  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed! !");
    isActive = false;
    return;
  }
  Serial.println("BMP180 initialization successful!");
}

double* bmp180_data() {
    static double data[3] = {0, 0, 0};
    if (!isActive) { 
        Serial.println("BMP180 is not working!");
        return data;
    }
    Serial.println("altitude: " + String(bmp.readAltitude()));
    Serial.println("temperature: " + String(bmp.readTemperature()));
    Serial.println("pressure: " + String((float)bmp.readPressure()/1000));
    // Lưu trữ dữ liệu vào mảng
    data[0] = bmp.readAltitude() ;
    data[1] = bmp.readTemperature();
    data[2] = (float)bmp.readPressure()/1000;
    return data;
}