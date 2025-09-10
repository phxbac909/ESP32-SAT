#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Khởi tạo điều khiển động cơ
void motor_init();

// Đặt tốc độ động cơ (0-255)
void motor_set_speed(int speed);

// Dừng động cơ
void motor_stop();

// Lấy tốc độ PWM hiện tại
int motor_get_current_speed();

#endif