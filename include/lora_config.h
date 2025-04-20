#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <SPI.h>
#include <LoRa.h>



void lora_begin();
void lora_receive_packet();
void lora_send_data(String data);

#endif