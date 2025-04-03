#ifndef ETHERNET_H
#define ETHERNET_H

#include <stdint.h>

extern  uint8_t MAC_ADDRESS[6];  
extern const int ETHERNET_CS_PIN;     
extern const int ETHERNET_SPI_CLK;    
extern const int ETHERNET_SPI_MOSI;   
extern const int ETHERNET_SPI_MISO;   
extern const int ETHERNET_RESET;   
extern const int ETHERNET_LAN_INT ;   

void startEthernet();

#endif