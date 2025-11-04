#ifndef PID_EULER_H
#define PID_EULER_H

#include <Arduino.h>

// Function declarations
void pid_euler_receive_command(int roll_command, int pitch_command);
void pid_euler_stop_task();
void pid_euler_set_base_throttle(int throttle);
void pid_euler_set_roll_pid(double kp, double ki, double kd);
void pid_euler_set_pitch_pid(double kp, double ki, double kd);
void pid_euler_init(void);

#endif