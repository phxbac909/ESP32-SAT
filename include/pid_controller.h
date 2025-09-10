#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>

// Khởi tạo bộ điều khiển PID
void pid_init();

// Cập nhật PID với giá trị đầu vào mới
float pid_update(float input_rpm);

// Xử lý lệnh điều khiển
void pid_handle_command(const String& command);

// Lấy tốc độ mục tiêu hiện tại
float pid_get_target();

// Lấy giá trị đầu ra PID
float pid_get_output();

// Lấy RPM hiện tại
float pid_get_current_rpm();

// Đặt thông số PID
void pid_set_parameters(double Kp, double Ki, double Kd);

// Đặt tốc độ mục tiêu
void pid_set_target(float target_rpm) ;

// Hàm cho task PID
void pid_task(void *parameter);

// Hàm cho task PID tuning sample
void pid_tunning_sample(void *parameter);

#endif