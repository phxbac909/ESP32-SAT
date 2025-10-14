#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <SPI.h>
#include <LoRa.h>

void lora_init();
void lora_send_data(String data);
void lora_send_packet();
String lora_receive_packet();
void lora_add_data(int value, size_t num_bytes) ;
void lora_add_all_data(const double* values, size_t num_values);
double gps_get_latitude();
double gps_get_longitude();
char lora_receive_command();

#endif