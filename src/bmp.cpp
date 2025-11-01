#include "bmp.h"
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>

Adafruit_BMP280 bmp;
SimpleKalmanFilter pressureKalman(0.5, 0.5, 0.5);
bool bmp_initialized = false;

bool bmp_init() {
    if (!bmp.begin(0x76)) {
        Serial.println("BMP280 not found!");
        return false;
    }
    
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16, 
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    
    // Làm nóng bộ lọc
    for (int i = 0; i < 10; i++) {
        bmp_altitude();
        delay(50);
    }
    
    bmp_initialized = true;
    Serial.println("BMP280 ready!");
    return true;
}

float bmp_altitude() {
    if (!bmp_initialized) return 0.0f;
    
    float pressure = bmp.readPressure() / 100.0f;
    if (pressure <= 0) return 0.0f;
    
    float filtered_pressure = pressureKalman.updateEstimate(pressure);
    float altitude = 44330.0f * (1.0f - pow(filtered_pressure / 1013.25f, 0.1903f));
    
    return altitude;
}

float bmp_pressure(){
        
    if (!bmp_initialized) return 0.0f;
    
    float pressure = bmp.readPressure() / 100.0f;
    if (pressure <= 0) return 0.0f;
    
    float filtered_pressure = pressureKalman.updateEstimate(pressure);

    return pressure;
}
float bmp_temperature() {
    if (!bmp_initialized) {
        Serial.println("BMP280 not initialized!");
        return 0.0;
    }
    
    float temperature = bmp.readTemperature();
    if (isnan(temperature)) {
        Serial.println("Error reading temperature!");
        return 0.0;
    }
    
    
    return temperature;
}