#include "pid_altitude.h"
#include "mpu6050.h"
#include "motor.h"
#include "PID_v1.h"

// PID tuning parameters - Có thể update từ bên ngoài
double PID_ALTITUDE_KP = 2.0;
double PID_ALTITUDE_KI = 0.1;
double PID_ALTITUDE_KD = 0.5;



// PID variables
double altitude_input = 0;      // Current accel Z value
double altitude_output = 0;     // PID output (throttle correction)
double altitude_setpoint = 0.88;
 // Target: 1g (hover)
// PID objects
PID altitude_pid(&altitude_input, &altitude_output, &altitude_setpoint, 
    PID_ALTITUDE_KP, PID_ALTITUDE_KI, PID_ALTITUDE_KD, 0); // DIRECT = 0

// Throttle control
static int altitude_command_throttle = 0;    // -100 to +100
static int altitude_base_throttle = 1000;    // Base PWM (1000) - GIỮ NGUYÊN
static int altitude_final_throttle_pwm = 1000; // Final PWM to motors
static bool altitude_hold_enabled = false;

// Task handle
static TaskHandle_t pid_altitude_task_handle = NULL;


void pid_altitude_receive_command(int command) {
    // Limit command to -100 to +100
    altitude_command_throttle = constrain(command, -100, 100);
    
    if (altitude_command_throttle == 0) {
        // Enable altitude hold when command is 0
        altitude_hold_enabled = true;
        Serial.println("Altitude Hold: ENABLED");
        
        // Cập nhật base throttle với giá trị hiện tại khi chuyển sang hold
        altitude_base_throttle = altitude_final_throttle_pwm;
    } else {
        // Disable altitude hold when manual command received
        altitude_hold_enabled = false;
        Serial.print("Manual Throttle Command: ");
        Serial.println(altitude_command_throttle);
    }
}

void pid_altitude_stop_task(){
    vTaskSuspend(pid_altitude_task_handle);
    motor_stop();
}


void pid_altitude_compute(void) {
    // Get current accelerometer Z
    altitude_input = mpu6050_accel_z();
    
    // Compute PID
    altitude_pid.Compute();
}

void pid_altitude_task(void *parameter) {
    while(1) {
        if (altitude_command_throttle == 0 && altitude_hold_enabled) {
            // Altitude Hold Mode - Use PID to maintain hover
            pid_altitude_compute();
            
            // Apply PID correction to base throttle
            altitude_final_throttle_pwm = altitude_base_throttle + (int)altitude_output;
            
            // // Debug output (optional)
            // static unsigned long last_debug = 0;
            // if (millis() - last_debug > 1000) {
            //     Serial.print("Alt Hold - AccelZ: ");
            //     Serial.print(altitude_input, 3);
            //     Serial.print(" | Correction: ");
            //     Serial.print(altitude_output, 1);
            //     Serial.print(" | Base: ");
            //     Serial.print(altitude_base_throttle);
            //     Serial.print(" | Final PWM: ");
            //     Serial.println(altitude_final_throttle_pwm);
            //     last_debug = millis();
            // }
        } else {
            // Manual Throttle Mode - Direct command control
            // Map command (-100 to +100) to PWM OFFSET (không thay đổi altitude_base_throttle)
            int throttle_offset = map(altitude_command_throttle, -100, 100, -800, 800);
            altitude_final_throttle_pwm = altitude_base_throttle + throttle_offset;
            
            // Debug output
            // static unsigned long last_manual_debug = 0;
            // if (millis() - last_manual_debug > 1000) {
            //     Serial.print("Manual - Command: ");
            //     Serial.print(altitude_command_throttle);
            //     Serial.print(" | Offset: ");
            //     Serial.print(throttle_offset);
            //     Serial.print(" | Base: ");
            //     Serial.print(altitude_base_throttle);
            //     Serial.print(" | Final PWM: ");
            //     Serial.println(altitude_final_throttle_pwm);
            //     last_manual_debug = millis();
            // }
        }
        
        // Constrain PWM output
        altitude_final_throttle_pwm = constrain(altitude_final_throttle_pwm, 1000, 2000);
        
        // Apply throttle to motors
        motor_set_throttle(altitude_final_throttle_pwm);
        
        vTaskDelay(20 / portTICK_PERIOD_MS); // 50Hz update rate
    }
}

int pid_altitude_get_current_throttle(void) {
    return altitude_command_throttle;
}

int pid_altitude_get_final_throttle(void) {
    return altitude_final_throttle_pwm;
}

void pid_altitude_set_base_throttle(int throttle) {
    altitude_base_throttle = constrain(throttle, 1000, 2000);
    Serial.print("Altitude Base Throttle Set to: ");
    Serial.println(altitude_base_throttle);
}

int pid_altitude_get_base_throttle(void) {
    return altitude_base_throttle;
}

bool pid_altitude_is_hold_enabled(void) {
    return altitude_hold_enabled;
}

void pid_altitude_update_tunings(void) {
    // Update PID tunings với giá trị mới từ biến global
    altitude_pid.SetTunings(PID_ALTITUDE_KP, PID_ALTITUDE_KI, PID_ALTITUDE_KD);
    Serial.print("Altitude PID Tunings Updated - Kp: ");
    Serial.print(PID_ALTITUDE_KP);
    Serial.print(" Ki: ");
    Serial.print(PID_ALTITUDE_KI);
    Serial.print(" Kd: ");
    Serial.println(PID_ALTITUDE_KD);
}

void pid_altitude_init(void) {
    // Initialize altitude PID với tuning parameters hiện tại
    altitude_pid.SetTunings(PID_ALTITUDE_KP, PID_ALTITUDE_KI, PID_ALTITUDE_KD);
    altitude_pid.SetMode(1); // AUTOMATIC = 1
    altitude_pid.SetOutputLimits(-100, 100); // Limit correction
    altitude_pid.SetSampleTime(20); // 50Hz
    
    // Reset variables
    altitude_command_throttle = 0;
    altitude_base_throttle = 1000;
    altitude_final_throttle_pwm = 1000;
    altitude_hold_enabled = false;
    
    // Create PID task
    xTaskCreate(
        pid_altitude_task,
        "PID Altitude Task",
        4096,
        NULL,
        1,
        &pid_altitude_task_handle
    );
    
    Serial.println("PID Altitude Controller Initialized");
    Serial.print("Tunings - Kp: ");
    Serial.print(PID_ALTITUDE_KP);
    Serial.print(" Ki: ");
    Serial.print(PID_ALTITUDE_KI);
    Serial.print(" Kd: ");
    Serial.println(PID_ALTITUDE_KD);
}
