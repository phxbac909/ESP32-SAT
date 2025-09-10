#include "motor_control.h"
#include "config.h"

static int current_speed = 0;

void motor_init() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  
  // Dừng động cơ ban đầu
  motor_stop();
}

void motor_set_speed(int speed) {
  speed = constrain(speed, 0, 255);
  current_speed = speed;
  
  if (speed == 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  }
  
  analogWrite(MOTOR_ENA, speed);
}

void motor_stop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
  current_speed = 0;
}

int motor_get_current_speed() {
  return current_speed;
}