// motor.cpp
#include <ESP32_Servo.h>
#include <Arduino.h>
#include <motor.h>

// Khai báo biến toàn cục
Servo motor1, motor2, motor3, motor4;
const int motorPins[4] = {25, 26, 27, 33};
bool motor_is_initialized = false;

boolean motor_is_active(){
    return motor_is_initialized;
};


// Hàm dừng tất cả motor (throttle 0)
void motor_stop() {
    if (!motor_is_initialized) return;
    // delay(2000); 
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
}

// Khởi tạo motor - KHÔNG hiệu chỉnh tự động
void motor_init() {
    if (motor_is_initialized) return;
    
    // Gắn motor vào các chân với pulse width range 1000-2000μs
    motor1.attach(motorPins[0], 1000, 2000);
    motor2.attach(motorPins[1], 1000, 2000);
    motor3.attach(motorPins[2], 1000, 2000);
    motor4.attach(motorPins[3], 1000, 2000);
    delay(2000); // Đợi ESC khởi động
    motor_is_initialized = true;
    // KHỞI TẠO Ở THROTTLE 0 - AN TOÀN
    motor_stop();

    Serial.println("Motor initialization successful!");
}

// Hàm hiệu chỉnh throttle range (CHỈ DÙNG KHI THAY ESC MỚI)
void motor_calibrate() {
    Serial.println("=== CẢNH BÁO: CHỈ HIỆU CHỈNH KHI THAY ESC MỚI ===");
    Serial.println("GỠ CÁNH QUẠT TRƯỚC KHI HIỆU CHỈNH!");
    Serial.println("Nhấn 'y' để tiếp tục...");
    
    while (!Serial.available()) delay(100);
    if (Serial.read() != 'y') {
        Serial.println("Hiệu chỉnh đã hủy");
        return;
    }
    
    Serial.println("Bắt đầu hiệu chỉnh...");
    
    // BƯỚC 1: Đặt throttle maximum TRƯỚC KHI lắp pin
    Serial.println("1. Đặt throttle maximum (2000μs)");
    motor1.writeMicroseconds(2000);
    motor2.writeMicroseconds(2000);
    motor3.writeMicroseconds(2000);
    motor4.writeMicroseconds(2000);
    delay(1000);
    
    // BƯỚC 2: LẮP PIN - ESC bắt đầu self-test
    Serial.println("2. LẮP PIN VÀO ESC NGAY BÂY GIỜ!");
    Serial.println("   ESC sẽ phát tiếng '123' và beep theo số cell pin");
    delay(8000); // Chờ người dùng lắp pin và ESC self-test
    
    // BƯỚC 3: ESC phát 2 beep xác nhận maximum throttle
    Serial.println("3. ESC đã nhận maximum throttle (2 beep)");
    delay(2000);
    
    // BƯỚC 4: Đặt throttle minimum trong 5 giây
    Serial.println("4. Đặt throttle minimum (1000μs)");
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
    delay(1000);
    
    // BƯỚC 5: ESC xác nhận bằng beep dài
    Serial.println("5. ESC xác nhận bằng beep dài - hiệu chỉnh hoàn tất!");
    delay(2000);
    
    Serial.println("✅ HIỆU CHỈNH THÀNH CÔNG!");
    Serial.println("✅ Tháo pin và lắp cánh quạt trước khi sử dụng");
}

void motor_receive_command(String command){
    if (!motor_is_initialized) {
        motor_init();
    }
    
    // Tách chuỗi command thành các số
    int pulseValues[4] = {1000, 1000, 1000, 1000};
    int index = 0;
    String temp = "";
    
    for (unsigned int i = 0; i < command.length(); i++) {
        if (command[i] == ' ') {
            if (temp != "" && index < 4) {
                pulseValues[index] = temp.toInt();
                index++;
                temp = "";
            }
        } else {
            temp += command[i];
        }
    }
    
    // Xử lý số cuối cùng
    if (temp != "" && index < 4) {
        pulseValues[index] = temp.toInt();
    }
    
    // Đảm bảo giá trị pulse nằm trong khoảng an toàn 1000-2000
    for (int i = 0; i < 4; i++) {
        if (pulseValues[i] < 1000) pulseValues[i] = 1000;
        if (pulseValues[i] > 2000) pulseValues[i] = 2000;
    }
    
    // Áp dụng pulse width cho các motor
    motor1.writeMicroseconds(pulseValues[0]);
    motor2.writeMicroseconds(pulseValues[1]);
    motor3.writeMicroseconds(pulseValues[2]);
    motor4.writeMicroseconds(pulseValues[3]);
    
    // Debug
    Serial.print("Motor pulses: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(pulseValues[i]);
        Serial.print(" ");
    }
    Serial.println();

}



void motor_rotate(int throttle , float roll, float pitch, float yaw ) {
    motor1.writeMicroseconds((int)round(throttle - roll + pitch + yaw));
    motor2.writeMicroseconds((int)round(throttle - roll - pitch - yaw));
    motor3.writeMicroseconds((int)round(throttle + roll - pitch + yaw));
    motor4.writeMicroseconds((int)round(throttle + roll + pitch - yaw));
}

void motor_set_throttle(int throttle){
    motor1.writeMicroseconds(throttle);
    motor2.writeMicroseconds(throttle);
    motor3.writeMicroseconds(throttle);
    motor4.writeMicroseconds(throttle);
}



void motor_detach() {
    motor_stop();
    delay(100);
    
    motor1.detach();
    motor2.detach();
    motor3.detach();
    motor4.detach();
    
    motor_is_initialized = false;
    Serial.println("Motor đã được giải phóng");
}