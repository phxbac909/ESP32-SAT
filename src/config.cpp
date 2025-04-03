#include "config.h"


// Định nghĩa các biến
const char* SSID = "Disconnect";
const char* PASSWORD = "254/101/17/24";
const int SERVER_PORT = 8080;
const char* TARGET_IP = "192.168.1.109";
const int TARGET_PORT = 8081;
const char* BT_NAME = "ESP32_BT";
 uint8_t MAC_ADDRESS[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};  // MAC tùy chỉnh
const int ETHERNET_CS_PIN = 20;     // Chân CS 
const int ETHERNET_SPI_CLK = 3;    // Chân SPI CLK 
const int ETHERNET_SPI_MOSI = 9;   // Chân SPI MOSI
const int ETHERNET_SPI_MISO = 46;   // Chân SPI MISO
