#include "pid_euler.h"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h" // Nhúng thư viện class PID
#include "esp32_now.h"
#include "data_struct.h"
#include <Arduino.h>

// --- TUNING PARAMETERS ---
// VÒNG GÓC (ANGLE) - Chỉ dùng P để tính toán tốc độ quay mong muốn (Target Rate)
// Bỏ hoàn toàn I và D ở vòng này.
double PID_ROLL_KP = 2.0;  double PID_ROLL_KI = 0.0;  double PID_ROLL_KD = 0.0;
double PID_PITCH_KP = 2.0; double PID_PITCH_KI = 0.0; double PID_PITCH_KD = 0.0;
double PID_YAW_KP = 2.0;   double PID_YAW_KI = 0.0;   double PID_YAW_KD = 0.0;

// VÒNG TỐC ĐỘ (RATE) - Đây mới là nơi thực hiện việc giữ thăng bằng
// P lớn hơn để phản ứng mạnh với gió/quán tính. D để hãm phanh chống lắc.
double RATE_ROLL_KP = 0.6;  double RATE_ROLL_KI = 0.0;  double RATE_ROLL_KD = 0.03;
double RATE_PITCH_KP = 0.6; double RATE_PITCH_KI = 0.0; double RATE_PITCH_KD = 0.03;
double RATE_YAW_KP = 0.6;   double RATE_YAW_KI = 0.0;   double RATE_YAW_KD = 0.0;

// --- KHỞI TẠO CÁC OBJECT PID ---
static PID pid_angle_roll;
static PID pid_angle_pitch;
static PID pid_angle_yaw;

static PID pid_rate_roll;
static PID pid_rate_pitch;
static PID pid_rate_yaw;

// --- CONTROL VARIABLES ---
static float roll_cmd = 0;
static float pitch_cmd = 0;
static float yaw_setpoint = 0; 
static int base_throttle = 1000;

static TaskHandle_t pid_euler_task_handle = NULL;

void pid_euler_set_base_throttle(int throttle) {
    base_throttle = constrain(throttle, 1000, 2000);
}

void pid_euler_set_roll_pid(double kp, double ki, double kd) {
    PID_ROLL_KP = kp; PID_ROLL_KI = ki; PID_ROLL_KD = kd;
    pid_angle_roll.setTunings(kp, ki, kd);
}

void pid_euler_set_pitch_pid(double kp, double ki, double kd) {
    PID_PITCH_KP = kp; PID_PITCH_KI = ki; PID_PITCH_KD = kd;
    pid_angle_pitch.setTunings(kp, ki, kd);
}

void pid_euler_set_pitch(int pitch){ pitch_cmd = constrain(pitch, -45, 45); }
void pid_euler_set_roll(int roll){ roll_cmd = constrain(roll, -45, 45); }
void pid_euler_set_yaw_target(int current_yaw_deg) { yaw_setpoint = current_yaw_deg; }

void pid_euler_stop_task() {
    vTaskSuspend(pid_euler_task_handle);
    motor_set_pulse(1000, 1000, 1000, 1000);
}

void pid_euler_task(void *parameter) {
    int log_counter = 0;
    imu_data_t current_imu; 
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 4 / portTICK_PERIOD_MS; // 250Hz -> dt = 4ms
    const float dt = 0.004f; 
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    IMU_Update_And_Read(&current_imu);
    yaw_setpoint = current_imu.yaw;

    while(1) {
        IMU_Update_And_Read(&current_imu);

        // Safety Stop
        if (abs(current_imu.roll) >= 45 || abs(current_imu.pitch) >= 45) {
            motor_stop();
            
            // Dùng phương thức reset() của object
            pid_rate_roll.reset(); 
            pid_rate_pitch.reset();
            pid_rate_yaw.reset();
            
            vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS); 
            continue;
        }
        
        // 1. TÍNH TOÁN VÒNG NGOÀI (ANGLE)
        float target_rate_roll = pid_angle_roll.compute(roll_cmd, current_imu.roll, dt);
        float target_rate_pitch = pid_angle_pitch.compute(pitch_cmd, current_imu.pitch, dt);
        float target_rate_yaw = pid_angle_yaw.compute(yaw_setpoint, current_imu.yaw, dt);

        // 2. TÍNH TOÁN VÒNG TRONG (RATE)
        float out_roll = pid_rate_roll.compute(target_rate_roll, current_imu.gyro_roll, dt);
        float out_pitch = pid_rate_pitch.compute(target_rate_pitch, current_imu.gyro_pitch, dt);
        float out_yaw = pid_rate_yaw.compute(target_rate_yaw, current_imu.gyro_yaw, dt);

        // 3. MIXING MOTOR
        int pwm1 = base_throttle - (int)out_pitch + (int)out_roll + (int)out_yaw; 
        int pwm2 = base_throttle + (int)out_pitch + (int)out_roll - (int)out_yaw; 
        int pwm3 = base_throttle + (int)out_pitch - (int)out_roll + (int)out_yaw;
        int pwm4 = base_throttle - (int)out_pitch - (int)out_roll - (int)out_yaw; 
        
        if (base_throttle < 1050) {
            pwm1 = 1000; pwm2 = 1000; pwm3 = 1000; pwm4 = 1000;
            
            pid_rate_roll.reset();
            pid_rate_pitch.reset();
            pid_rate_yaw.reset();
            
            yaw_setpoint = current_imu.yaw;
        } else {
            pwm1 = constrain(pwm1, 1000, 2000);
            pwm2 = constrain(pwm2, 1000, 2000);
            pwm3 = constrain(pwm3, 1000, 2000);
            pwm4 = constrain(pwm4, 1000, 2000);
        }
        
        motor_set_pulse(pwm1, pwm2, pwm3, pwm4);
       
        // 4. LOGGING (25Hz)
        log_counter++;
        if (log_counter >= 10) { 
            DroneLog log;
            log.roll_target = roll_cmd;
            log.pitch_target = pitch_cmd;
            log.roll = current_imu.roll;      
            log.pitch = current_imu.pitch;     
            log.pwm1 = pwm1;
            log.pwm2 = pwm2;
            log.pwm3 = pwm3;
            log.pwm4 = pwm4;
            uint8_t* data = (uint8_t*)&log;
            esp32_now_send(data, sizeof(DroneLog));
            log_counter = 0;
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void pid_euler_init(void) {
    // Gọi phương thức init() cho từng object
    pid_angle_roll.init(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, -200, 200);
    pid_angle_pitch.init(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, -200, 200);
    pid_angle_yaw.init(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, -200, 200);

    pid_rate_roll.init(RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD, -120, 120);
    pid_rate_pitch.init(RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD, -120, 120);
    pid_rate_yaw.init(RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD, -120, 120);
    
    roll_cmd = 0;
    pitch_cmd = 0;
    yaw_setpoint = 0;
    base_throttle = 1000;
    
    xTaskCreate(pid_euler_task, "PID Euler Task", 4096, NULL, 5, &pid_euler_task_handle);
}