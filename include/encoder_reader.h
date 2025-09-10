#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

// Khởi tạo encoder
void encoder_init();

// Đọc số xung encoder
long encoder_get_count();

// Tính toán RPM
float encoder_calculate_rpm(unsigned long elapsed_time);

// Reset bộ đếm encoder
void encoder_reset();

// Kiểm tra encoder (debug)
void encoder_test();

// Xác định chiều quay (không khả thi với 1 chân)
bool encoder_get_direction();

#endif