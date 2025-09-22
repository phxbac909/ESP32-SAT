#ifndef CONFIG_H
#define CONFIG_H

// Chân kết nối LM298N
#define MOTOR_ENA 27
#define MOTOR_IN1 26
#define MOTOR_IN2 25

// Chân kết nối encoder
#define ENCODER_A 12
#define ENCODER_B 13

// Cài đặt PID
#define PID_SAMPLE_TIME 100  // ms
#define PID_MIN_OUTPUT 150
#define PID_MAX_OUTPUT 800

// Tốc độ mục tiêu mặc định (RPM)
#define DEFAULT_TARGET_RPM 5

#endif