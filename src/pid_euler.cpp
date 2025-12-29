#include "pid_euler.h"
#include "mpu6050.h"
#include "motor.h"
#include "PID_v1.h"
#include "esp32_now.h"
#include "data_struct.h"
#include "config.h"


// PID tuning parameters
double PID_ROLL_KP = 1.0;
double PID_ROLL_KI = 0;
double PID_ROLL_KD = 0;

double PID_PITCH_KP = 1.0;
double PID_PITCH_KI = 0;
double PID_PITCH_KD = 0;

// PID variables
double roll_input = 0;
double roll_output = 0;
double roll_setpoint = 0.0;

double pitch_input = 0;
double pitch_output = 0;
double pitch_setpoint = 0.0;

// PID objects
PID roll_pid(&roll_input, &roll_output, &roll_setpoint, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, 0);
PID pitch_pid(&pitch_input, &pitch_output, &pitch_setpoint, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 0);

// Control variables
static int roll_command = 0;
static int pitch_command = 0;
static int base_throttle = 1000;

bool write_log = true;

// Task handle
static TaskHandle_t pid_euler_task_handle = NULL;

void pid_euler_set_base_throttle(int throttle) {
    base_throttle = constrain(throttle, 1000, 2000);
}

void pid_euler_set_roll_pid(double kp, double ki, double kd) {
    PID_ROLL_KP = kp;
    PID_ROLL_KI = ki;
    PID_ROLL_KD = kd;
    roll_pid.SetTunings(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
}

void pid_euler_set_pitch_pid(double kp, double ki, double kd) {
    PID_PITCH_KP = kp;
    PID_PITCH_KI = ki;
    PID_PITCH_KD = kd;
    pitch_pid.SetTunings(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
}

void pid_euler_set_pitch(int pitch){
    pitch_command = constrain(pitch, -100, 100);
}

void pid_euler_set_roll(int roll){
    roll_command = constrain(roll, -100, 100);
}


void pid_euler_stop_task() {
    vTaskSuspend(pid_euler_task_handle);
    motor_set_pulse(1000, 1000, 1000, 1000);
}

void pid_euler_task(void *parameter) {
    while(1) {

        roll_input = mpu6050_roll();
        pitch_input = mpu6050_pitch();
        
        roll_setpoint = roll_command;
        pitch_setpoint = pitch_command;
        
        // Compute PID
        roll_pid.Compute();
        pitch_pid.Compute();
        
        // Calculate motor PWM using mixing
        int pwm1 = base_throttle - (int)pitch_output + (int)roll_output; // FRONT_RIGHT
        int pwm2 = base_throttle - (int)pitch_output - (int)roll_output; // FRONT_LEFT  
        int pwm3 = base_throttle + (int)pitch_output - (int)roll_output; // REAR_LEFT
        int pwm4 = base_throttle + (int)pitch_output + (int)roll_output; // REAR_RIGHT
        
        // Constrain and set motor PWM
        pwm1 = constrain(pwm1, 1000, 2000);
        pwm2 = constrain(pwm2, 1000, 2000);
        pwm3 = constrain(pwm3, 1000, 2000);
        pwm4 = constrain(pwm4, 1000, 2000);
        
        motor_set_pulse(pwm1, pwm2, pwm3, pwm4);

        if (write_log){
            DroneLog log ;
            log.roll_target = roll_command;
            log.pitch_target = pitch_command;
            log.roll = mpu6050_roll();
            log.pitch = mpu6050_roll();
            log.pwm1 = pwm1;
            log.pwm2 = pwm2;
            log.pwm3 = pwm3;
            log.pwm4 = pwm4;
            uint8_t* data = (uint8_t*)&log;
            esp32_now_send(data,sizeof(DroneLog));
        }
        write_log= !write_log;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void pid_euler_init(void) {
    // Initialize PID controllers
    roll_pid.SetMode(1);
    roll_pid.SetOutputLimits(-200, 200);
    roll_pid.SetSampleTime(20);
    
    pitch_pid.SetMode(1);
    pitch_pid.SetOutputLimits(-200, 200);
    pitch_pid.SetSampleTime(20);
    
    // Reset variables
    roll_command = 0;
    pitch_command = 0;
    base_throttle = 1000;
    
    // Create task
    xTaskCreate(
        pid_euler_task,
        "PID Euler Task",
        4096,
        NULL,
        2,
        &pid_euler_task_handle
    );
}