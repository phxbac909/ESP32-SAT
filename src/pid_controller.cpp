#include "pid_controller.h"
#include "config.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Biến PID
static double pid_input, pid_output, pid_setpoint;
static PID myPID(&pid_input, &pid_output, &pid_setpoint, 0.3, 0.05, 0.005, DIRECT);
// static PID myPID(&pid_input, &pid_output, &pid_setpoint, 0.013005, 0.000029, 1.301311, DIRECT);
static float current_rpm = 0;
static unsigned long last_encoder_time = 0;

void pid_init() {
  // Cấu hình PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(PID_SAMPLE_TIME);
  myPID.SetOutputLimits(PID_MIN_OUTPUT, PID_MAX_OUTPUT);
  
  // Đặt giá trị mặc định
  pid_setpoint = DEFAULT_TARGET_RPM;
  last_encoder_time = millis();
}

float pid_update(float input_rpm) {
  current_rpm = input_rpm;
  pid_input = input_rpm;
  myPID.Compute();
  return pid_output;
}

// Task xử lý PID
void pid_task(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(PID_SAMPLE_TIME);
  
  for (;;) {
    // Tính toán RPM và cập nhật PID
    unsigned long current_time = millis();
    unsigned long elapsed_time = current_time - last_encoder_time;
    
    if (elapsed_time > 0) {
      float rpm = encoder_calculate_rpm(elapsed_time);
    
      float output = pid_update(rpm);
      motor_set_speed((int)output);
      
      // Serial.print("RPM: ");
      // Serial.println(rpm);
      // motor_set_speed(800);

      last_encoder_time = current_time;
      encoder_reset(); // Reset bộ đếm sau mỗi lần tính toán
    }
    
    // Chờ đến chu kỳ tiếp theo
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task xử lý PID
void pid_tunning_sample(void *parameter) {
}


void pid_handle_command(const String& command) {
  if (command.length() == 0) return;
  
  char cmd_char = command.charAt(0);
  
  switch (cmd_char) {
    case '+':
      pid_set_target(pid_get_target() + 10);
      Serial.print("Increased speed to: ");
      Serial.println(pid_get_target());
      break;
      
    case '-':
      {
        float new_target = pid_get_target() - 10;
        if (new_target < 0) new_target = 0;
        pid_set_target(new_target);
        Serial.print("Decreased speed to: ");
        Serial.println(pid_get_target());
      }
      break;
      
    case '0':
      pid_set_target(0);
      motor_stop();
      Serial.println("Motor stopped");
      break;
      
    case 'r':
      encoder_reset();
      Serial.println("Encoder reset");
      break;
      
    case 's':
      Serial.print("Current RPM: ");
      Serial.print(pid_get_current_rpm());
      Serial.print(" | Target RPM: ");
      Serial.print(pid_get_target());
      Serial.print(" | PWM: ");
      Serial.println(motor_get_current_speed());
      break;
      
    default:
      {
        float target = command.toFloat();
        if (target >= 0) {
          pid_set_target(target);
          Serial.print("Target speed set to: ");
          Serial.println(target);
        }
      }
      break;
  }
}

void pid_set_target(float target_rpm) {
  pid_setpoint = target_rpm;
}

float pid_get_target() {
  return pid_setpoint;
}

float pid_get_output() {
  return pid_output;
}

float pid_get_current_rpm() {
  return current_rpm;
}

void pid_set_parameters(double Kp, double Ki, double Kd) {
  myPID.SetTunings(Kp, Ki, Kd);
}