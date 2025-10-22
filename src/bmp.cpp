#include "bmp.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>

// Using SimpleKalmanFilter for noise filtering
#include <SimpleKalmanFilter.h>

// Define variables here instead of in header
Adafruit_BMP280 bmp;
SimpleKalmanFilter pressureKalman(1, 1, 0.01);
SimpleKalmanFilter tempKalman(1, 1, 0.01);

bool bmp_initialized = false;
float sea_level_pressure = 1013.25; // Default sea level pressure in hPa

bool bmp_init() {
    if (!bmp.begin(0x68)) {  // BMP280 I2C address is usually 0x76 or 0x77
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        bmp_initialized = false;
        return false;
    }
    
    /* Default settings from library */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    
    // Read initial sea level pressure (average of first few readings)
    float sum_pressure = 0;
    int readings = 10;
    
    for (int i = 0; i < readings; i++) {
        float pressure = bmp.readPressure() / 100.0; // Convert to hPa
        if (pressure > 0) {
            sum_pressure += pressure;
            delay(100);
        }
    }
    
    sea_level_pressure = sum_pressure / readings;
    Serial.print("Sea level pressure set to: ");
    Serial.print(sea_level_pressure);
    Serial.println(" hPa");
    
    bmp_initialized = true;
    Serial.println("BMP280 initialized successfully!");
    return true;
}

float bmp_altitude() {
    if (!bmp_initialized) {
        Serial.println("BMP280 not initialized!");
        return 0.0;
    }
    
    float pressure = bmp.readPressure() / 100.0; // Convert to hPa
    if (pressure <= 0) {
        Serial.println("Error reading pressure!");
        return 0.0;
    }
    
    // Apply Kalman filter to pressure reading
    float filtered_pressure = pressureKalman.updateEstimate(pressure);
    
    // Calculate altitude using barometric formula with filtered pressure
    float altitude = 44330.0 * (1.0 - pow(filtered_pressure / sea_level_pressure, 0.1903));
    
    return altitude;
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
    
    // Apply Kalman filter to temperature reading
    float filtered_temperature = tempKalman.updateEstimate(temperature);
    
    return filtered_temperature;
}