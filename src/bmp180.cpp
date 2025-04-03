#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "bmp180.h"

Adafruit_BMP085 bmp;

void bmp180Service(void *pvParameters) {
  if (!bmp.begin(0x77)) {
    Serial.println("Không tìm thấy BMP180!");
    while (1) vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  Serial.println("BMP180 đã được tìm thấy!");

  while (1) {
    Serial.println("=== BMP180 ===");
    Serial.print("Nhiệt độ: "); Serial.print(bmp.readTemperature()); Serial.println(" °C");
    Serial.print("Áp suất: "); Serial.print(bmp.readPressure() / 100.0); Serial.println(" hPa");
    Serial.print("Độ cao: "); Serial.print(bmp.readAltitude(1013.25)); Serial.println(" m");

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Đợi 2 giây
  }
}

void startBMP180() {
  xTaskCreate(
    bmp180Service,   // Hàm service
    "BMP180Service", // Tên task
    4096,            // Kích thước stack
    NULL,            // Tham số truyền vào
    1,               // Độ ưu tiên
    NULL             // Handle của task
  );
}