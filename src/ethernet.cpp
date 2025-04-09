#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>  
#include "ethernet_config.h"

uint8_t MAC_ADDRESS[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};  
const int ETHERNET_CS_PIN = 20;     
const int ETHERNET_SPI_CLK = 3; 
const int ETHERNET_SPI_MOSI = 9;   
const int ETHERNET_SPI_MISO = 46;   

TaskHandle_t ethernetTaskHandle = NULL;
EthernetClient ethernetClient;

void ethernetTask(void *parameter) {
  while (true) {
    if (Ethernet.linkStatus() == LinkON) {
      Serial.println("Ethernet link is ON");
      Serial.print("IP address: ");
      Serial.println(Ethernet.localIP());
      if (ethernetClient.connect("google.com", 80)) {
        Serial.println("Connected to google.com");
        ethernetClient.println("GET / HTTP/1.1");
        ethernetClient.println("Host: google.com");
        ethernetClient.println("Connection: close");
        ethernetClient.println();
        while (ethernetClient.connected()) {
          if (ethernetClient.available()) {
            String line = ethernetClient.readStringUntil('\n');
            Serial.println(line);
          }
        }
        ethernetClient.stop();
        Serial.println("Disconnected from google.com");
      } else {
        Serial.println("Failed to connect to google.com");
      }
    } else {
      Serial.println("Ethernet link is OFF");
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void startEthernet() {
  Serial.println("Initializing Ethernet...");
  SPI.begin(ETHERNET_SPI_CLK, ETHERNET_SPI_MISO, ETHERNET_SPI_MOSI, ETHERNET_CS_PIN); // Khởi tạo SPI
  Ethernet.init(ETHERNET_CS_PIN);
  delay(1000); // Đợi SPI ổn định

  Serial.println("Starting Ethernet with MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(MAC_ADDRESS[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (Ethernet.begin(const_cast<uint8_t*>(MAC_ADDRESS)) == 0) {
    Serial.println("Lấy IP từ DHCP thất bại, thử đặt IP tĩnh...");
    Ethernet.begin(const_cast<uint8_t*>(MAC_ADDRESS), IPAddress(192, 168, 1, 100));
    // In thêm thông tin debug
    Serial.print("Hardware status: ");
    Serial.println(Ethernet.hardwareStatus());
    Serial.print("Link status: ");
    Serial.println(Ethernet.linkStatus());
    // return;
  }
  Serial.println("Ethernet started");
  Serial.print("IP address: ");
  Serial.println(Ethernet.localIP());
  // xTaskCreate(ethernetTask, "EthernetTask", 8192, NULL, 1, &ethernetTaskHandle);
}