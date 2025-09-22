#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

// Khởi tạo encoder
void encoder_init();

// Tính toán RPM
float encoder_get_rpm();

#endif