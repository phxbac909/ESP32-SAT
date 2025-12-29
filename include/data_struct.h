#include <Arduino.h>

struct ControlData {
    byte id;
    int16_t speed;
};

struct PidEulerData {
    byte id;
    float kp;
    float ki;
    float kd;
};

struct StopSignal {
    byte id;
};

struct DroneLog {
    float roll_target;
    float pitch_target;
    float roll;
    float pitch;
    int pwm1;
    int pwm2;
    int pwm3;
    int pwm4;
}__attribute__((packed));