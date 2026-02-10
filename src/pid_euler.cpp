#include "pid_euler.h"
#include "mpu6050.h"
#include "motor.h"
#include "PID_v1.h"
#include "esp32_now.h"
#include "data_struct.h"
#include "config.h"

// --- TUNING PARAMETERS ---
// PID GÓC (Vòng ngoài - Stabilize): Output sẽ là Target Rate (độ/giây)
// Mặc định P nên thấp hơn Rate, I và D thường bằng 0 hoặc rất nhỏ
double PID_ROLL_KP = 0.2;  double PID_ROLL_KI = 0.0;  double PID_ROLL_KD = 0.003;
double PID_PITCH_KP = 0.2; double PID_PITCH_KI = 0.0; double PID_PITCH_KD = 0.003;
double PID_YAW_KP = 0.2;   double PID_YAW_KI = 0.0;   double PID_YAW_KD = 0.003;

// PID TỐC ĐỘ (Vòng trong - Rate): Output sẽ là PWM Correction
// Đây là vòng cần tune kỹ nhất.
double RATE_ROLL_KP = 0.15;  double RATE_ROLL_KI = 0;  double RATE_ROLL_KD = 0.0;
double RATE_PITCH_KP = 0.15; double RATE_PITCH_KI = 0; double RATE_PITCH_KD = 0.0;
double RATE_YAW_KP = 0.15;   double RATE_YAW_KI = 0;   double RATE_YAW_KD = 0.0;

// --- VARIABLES ---
// 1. ANGLE LOOP VARIABLES (Giữ nguyên tên cũ cho tương thích code ngoài)
double roll_input = 0, roll_output = 0, roll_setpoint = 0;
double pitch_input = 0, pitch_output = 0, pitch_setpoint = 0;
double yaw_input = 0, yaw_output = 0, yaw_setpoint = 0; // Thêm Yaw angle

// 2. RATE LOOP VARIABLES (Thêm mới)
double roll_rate_input = 0, roll_rate_output = 0, roll_rate_setpoint = 0;
double pitch_rate_input = 0, pitch_rate_output = 0, pitch_rate_setpoint = 0;
double yaw_rate_input = 0, yaw_rate_output = 0, yaw_rate_setpoint = 0;

// --- PID OBJECTS ---
// Vòng ngoài (Angle): Output giới hạn là tốc độ quay tối đa (ví dụ +/- 200 độ/giây)
PID roll_pid(&roll_input, &roll_output, &roll_setpoint, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, DIRECT);
PID pitch_pid(&pitch_input, &pitch_output, &pitch_setpoint, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yaw_pid(&yaw_input, &yaw_output, &yaw_setpoint, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

// Vòng trong (Rate): Output giới hạn là PWM adjustment (ví dụ +/- 400 xung)
PID roll_rate_pid(&roll_rate_input, &roll_rate_output, &roll_rate_setpoint, RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD, DIRECT);
PID pitch_rate_pid(&pitch_rate_input, &pitch_rate_output, &pitch_rate_setpoint, RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD, DIRECT);
PID yaw_rate_pid(&yaw_rate_input, &yaw_rate_output, &yaw_rate_setpoint, RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD, DIRECT);

// Control variables
static int roll_command = 0;
static int pitch_command = 0;
static int yaw_command = 0; // Thêm biến lưu hướng yaw
static int base_throttle = 1000;

// Task handle
static TaskHandle_t pid_euler_task_handle = NULL;

void pid_euler_set_base_throttle(int throttle) {
    base_throttle = constrain(throttle, 1000, 2000);
}

// Giữ nguyên hàm set PID cho vòng ngoài (để bạn dễ chỉnh P angle)
void pid_euler_set_roll_pid(double kp, double ki, double kd) {
    PID_ROLL_KP = kp; PID_ROLL_KI = ki; PID_ROLL_KD = kd;
    roll_pid.SetTunings(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
}

void pid_euler_set_pitch_pid(double kp, double ki, double kd) {
    PID_PITCH_KP = kp; PID_PITCH_KI = ki; PID_PITCH_KD = kd;
    pitch_pid.SetTunings(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
}

// Hàm setter angle giữ nguyên
void pid_euler_set_pitch(int pitch){
    pitch_command = constrain(pitch, -45, 45);
}

void pid_euler_set_roll(int roll){
    roll_command = constrain(roll, -45, 45);
}

// Thêm hàm set yaw nếu cần (giả sử yaw stick input)
void pid_euler_set_yaw_target(int current_yaw_deg) {
     yaw_setpoint = current_yaw_deg;
}

void pid_euler_stop_task() {
    vTaskSuspend(pid_euler_task_handle);
    motor_set_pulse(1000, 1000, 1000, 1000);
}

void pid_euler_task(void *parameter) {
    int log_counter = 0;
    
    // Khởi tạo Yaw setpoint bằng Yaw hiện tại khi bắt đầu để không bị xoay
    vTaskDelay(100 / portTICK_PERIOD_MS);
    yaw_setpoint = mpu6050_yaw();

    while(1) {
        // 1. ĐỌC DỮ LIỆU CẢM BIẾN
        // Góc (Angle) cho vòng ngoài
        roll_input = mpu6050_roll();
        pitch_input = mpu6050_pitch();
        yaw_input = mpu6050_yaw(); 
        
        // Tốc độ góc (Gyro Rate) cho vòng trong
        roll_rate_input = mpu6050_gyro_roll();
        pitch_rate_input = mpu6050_gyro_pitch();
        yaw_rate_input = mpu6050_gyro_yaw();

        // Debug
        DEBUG_PRINT("R: "); DEBUG_PRINT(roll_input); DEBUG_PRINT(" | P: "); DEBUG_PRINTLN(pitch_input);

        // Safety Stop (Góc nghiêng quá lớn)
        if (abs(roll_input) >= 45 || abs(pitch_input) >= 45) {
            motor_stop();
            // Reset PID khi ngã để tránh tích luỹ I
            roll_rate_output = 0; pitch_rate_output = 0; yaw_rate_output = 0;
            vTaskDelay(100/portTICK_PERIOD_MS);
            continue;
        }
        
        // 2. TÍNH TOÁN VÒNG NGOÀI (ANGLE / STABILIZE)
        roll_setpoint = roll_command;
        pitch_setpoint = pitch_command;
        // Yaw giữ nguyên setpoint (Heading hold) trừ khi có lệnh stick (chưa implement stick yaw ở đây)
        
        roll_pid.Compute();  // Output: roll_output (Desired Rate)
        pitch_pid.Compute(); // Output: pitch_output (Desired Rate)
        yaw_pid.Compute();   // Output: yaw_output (Desired Rate)
        
        // 3. TÍNH TOÁN VÒNG TRONG (RATE / ACRO)
        // Setpoint của vòng trong chính là Output của vòng ngoài
        roll_rate_setpoint = roll_output;
        pitch_rate_setpoint = pitch_output;
        yaw_rate_setpoint = yaw_output;

        roll_rate_pid.Compute();  // Output: roll_rate_output (PWM correction)
        pitch_rate_pid.Compute(); // Output: pitch_rate_output (PWM correction)
        yaw_rate_pid.Compute();   // Output: yaw_rate_output (PWM correction)


        // 4. MIXING (Đã sửa lại dấu cho đúng chiều vật lý)
        // Lưu ý: PID DIRECT mode -> Output dương nghĩa là muốn tăng góc/tốc độ theo chiều dương.
        
        // Pitch Output > 0 (Muốn ngóc đầu lên) -> Cần TĂNG motor trước (1,2), GIẢM motor sau (3,4)
        // Roll Output > 0 (Muốn nghiêng phải) -> Cần TĂNG motor trái (2,3), GIẢM motor phải (1,4)
        // Yaw Output > 0 (Muốn xoay phải/CW) -> Cần TĂNG motor quay CCW (2,4), GIẢM motor quay CW (1,3) (Phản lực)
        
        int r_out = (int)roll_rate_output;
        int p_out = (int)pitch_rate_output;
        int y_out = (int)yaw_rate_output;

        // Cấu hình Quad X chuẩn:
        // PWM1 (Front Right - CW):  +Pitch -Roll -Yaw
        // PWM2 (Front Left - CCW):  +Pitch +Roll +Yaw
        // PWM3 (Rear Left - CW):    -Pitch +Roll -Yaw
        // PWM4 (Rear Right - CCW):  -Pitch -Roll +Yaw
        int pwm2 = base_throttle + p_out - r_out + y_out; // FR (CW)
        int pwm3 = base_throttle + p_out + r_out - y_out; // FL (CCW)
        int pwm4 = base_throttle - p_out + r_out + y_out; // RL (CW)
        int pwm1 = base_throttle - p_out - r_out - y_out; // RR (CCW)

        
        // Safety: Nếu ga thấp thì không chạy PID Rate để tránh quay motor dưới đất
        if (base_throttle < 1050) {
            pwm1 = 1000; pwm2 = 1000; pwm3 = 1000; pwm4 = 1000;
            // Reset Integral term của vòng Rate để tránh windup
            roll_rate_pid.SetMode(MANUAL); roll_rate_pid.SetMode(AUTOMATIC);
            pitch_rate_pid.SetMode(MANUAL); pitch_rate_pid.SetMode(AUTOMATIC);
            yaw_rate_pid.SetMode(MANUAL); yaw_rate_pid.SetMode(AUTOMATIC);
        } else {
            pwm1 = constrain(pwm1, 1000, 2000);
            pwm2 = constrain(pwm2, 1000, 2000);
            pwm3 = constrain(pwm3, 1000, 2000);
            pwm4 = constrain(pwm4, 1000, 2000);
        }
        
        motor_set_pulse(pwm1, pwm2, pwm3, pwm4);
       

        // Logging
        log_counter++;
        if (log_counter >= 20) {
            DroneLog log;
            log.roll_target = roll_command;
            log.pitch_target = pitch_command;
            log.roll = roll_input;       // Góc đo được
            log.pitch = pitch_input;     // Góc đo được
            log.pwm1 = pwm1;
            log.pwm2 = pwm2;
            log.pwm3 = pwm3;
            log.pwm4 = pwm4;
            uint8_t* data = (uint8_t*)&log;
            esp32_now_send(data, sizeof(DroneLog));
            // Serial.print("R: "); Serial.print(roll_input); Serial.print(" | P: "); Serial.print(pitch_input);
            // Serial.print(" : ");
            // Serial.print(pwm2);
            // Serial.print("...");
            // Serial.print(pwm3);
            // Serial.print("...");
            // Serial.print(pwm4);
            // Serial.print("...");
            // Serial.print(pwm1);
            // Serial.println();
            log_counter = 0;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz Loop
    }
}

void pid_euler_init(void) {
    // 1. Init Vòng Angle (Outer) - Output là độ/giây
    // Max tilt rate = 200 deg/s
    roll_pid.SetMode(AUTOMATIC);
    roll_pid.SetOutputLimits(-200, 200); 
    roll_pid.SetSampleTime(10);
    
    pitch_pid.SetMode(AUTOMATIC);
    pitch_pid.SetOutputLimits(-200, 200);
    pitch_pid.SetSampleTime(10);

    yaw_pid.SetMode(AUTOMATIC);
    yaw_pid.SetOutputLimits(-200, 200);
    yaw_pid.SetSampleTime(10);

    // 2. Init Vòng Rate (Inner) - Output là PWM pulse adjustment
    // Max PWM correction = +/- 150us
   roll_rate_pid.SetMode(AUTOMATIC); 
    roll_rate_pid.SetOutputLimits(-120, 120); 
    roll_rate_pid.SetSampleTime(4);

    pitch_rate_pid.SetMode(AUTOMATIC); 
    pitch_rate_pid.SetOutputLimits(-120, 120); 
    pitch_rate_pid.SetSampleTime(4);

    yaw_rate_pid.SetMode(AUTOMATIC); 
    yaw_rate_pid.SetOutputLimits(-120, 120); 
    yaw_rate_pid.SetSampleTime(4);
    
    // Reset variables
    roll_command = 0;
    pitch_command = 0;
    yaw_command = 0;
    base_throttle = 1000;
    
    // Create task
    xTaskCreate(
        pid_euler_task,
        "PID Euler Task",
        4096, // Tăng stack size lên chút cho an toàn
        NULL,
        2,
        &pid_euler_task_handle
    );
}
