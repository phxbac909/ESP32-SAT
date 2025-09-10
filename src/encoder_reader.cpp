#include "encoder_reader.h"
#include <Arduino.h>
#include "config.h"
// Biến toàn cục
volatile long encoder_count = 0;
volatile unsigned long last_pulse_time = 0;
const int PULSES_PER_REVOLUTION = 1140;
const unsigned long STOP_TIMEOUT = 500000; // 0.5 giây timeout (microseconds)

// Hàm ngắt để cập nhật bộ đếm encoder
void update_encoder() {
  unsigned long current_time = micros();
  unsigned long time_since_last_pulse = current_time - last_pulse_time;
  
  // Lọc nhiễu cơ bản - bỏ qua các xung cách nhau quá gần
  if (time_since_last_pulse > 100) {
    encoder_count++;
    last_pulse_time = current_time;
  }
}

// Khởi tạo encoder
void encoder_init() {
  pinMode(ENCODER_A , INPUT_PULLUP); // ENCODER_A
  pinMode(ENCODER_B, INPUT_PULLUP); // ENCODER_B
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A ), update_encoder, RISING);
  
  Serial.println("Encoder initialized");
}


// Đọc số xung encoder
long encoder_get_count() {
  noInterrupts();
  long count = encoder_count;
  interrupts();
  return count;
}

// Tính toán RPM
float encoder_calculate_rpm(unsigned long elapsed_time) {
  noInterrupts();
  long count = encoder_count;
  interrupts();
  
  // Kiểm tra nếu động cơ đã dừng
  unsigned long current_time = micros();
  if (current_time - last_pulse_time > STOP_TIMEOUT) {
    return 0.0;
  }
  
  // Tính RPM (vòng/phút)
  if (elapsed_time > 0) {
    float revolutions = (float)count / PULSES_PER_REVOLUTION;
    float minutes = (float)elapsed_time / 60000.0; // chuyển ms thành phút
    return revolutions / minutes;
  }
  
  return 0.0;
}

// Reset bộ đếm encoder
void encoder_reset() {
  noInterrupts();
  encoder_count = 0;
  last_pulse_time = micros();
  interrupts();
}

// Kiểm tra encoder (debug)
void encoder_test() {
  Serial.println("=== ENCODER TEST ===");
  Serial.print("Current count: ");
  Serial.println(encoder_get_count());
  
  unsigned long current_time = micros();
  Serial.print("Time since last pulse: ");
  Serial.print(current_time - last_pulse_time);
  Serial.println(" us");
  
  Serial.print("Motor status: ");
  if (current_time - last_pulse_time > STOP_TIMEOUT) {
    Serial.println("STOPPED");
  } else {
    Serial.println("RUNNING");
  }
  Serial.println("====================");
}

// // Xác định chiều quay (không khả thi với 1 chân)
// bool encoder_get_direction() {
//   // Với encoder 1 chân, không thể xác định chiều quay
//   // Luôn trả về true (chiều thuận) làm giá trị mặc định
//   return true;
// }