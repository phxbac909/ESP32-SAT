#include <Arduino.h>
#include "mpu6050.h"
#include "bmp180.h"
#include "lora_config.h"
#include "gps.h"
#include <LiquidCrystal_I2C.h>
#include "config.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include "pid_controller.h"

// volatile int order = 0;


// void task_send_data_to_ground_station(void *pvParameters){
//   (void)pvParameters; // Bỏ qua tham số nếu không sử dụng

//   while (1){

//   Serial.println("Send data with order = " + String(order));
//   lora_add_data(order,1);
//   lora_add_all_data(bmp180_data(),3); 
//   lora_add_all_data(mpu6050_data(),3);

  
//   TinyGPSLocation location = get_gps_location();
//   // Hiển thị thông tin
//   if (location.isValid()) {
//      lora_add_data( (int) location.lng()*1000000,4 );
//      lora_add_data( (int) location.lat()*1000000,4 );
//   } else {
//      lora_add_data( 0,4 );
//      lora_add_data( 0,4 );
//   }
 
  
//   lora_send_packet();
  
//   order++;
  
//   Serial.println("----------------------------");
//   Serial.println("----------------------------");
  
//   vTaskDelay(5000 / portTICK_PERIOD_MS); // Đợi 5 giây 
//  }
 
// }

// void task_send_rpm_to_ground_station(void *pvParameters){
//   (void)pvParameters; // Bỏ qua tham số nếu không sử dụng
//  while (1){
//   float current_rpm = pid_get_current_rpm();
//   Serial.println("Current RPM: " + String(current_rpm));
//   lora_send_data(String(current_rpm));
//   vTaskDelay(3000 / portTICK_PERIOD_MS); // Đợi 2 giây 
//  }
 
// }

// void task_receive_data_from_ground_station(void *pvParameters){
//   (void)pvParameters; // Bỏ qua tham số nếu không sử dụng
//  while (1){
//   char command = lora_receive_command();
//   if (command != '\0') {
//       // pid_handle_command(String(command));
//   }
 
//   vTaskDelay(100 / portTICK_PERIOD_MS); // Đợi 100 ms để tránh quá tải CPU
//  }
 
// }

// // Task xử lý serial
// void serial_task(void *parameter) {
//   for (;;) {
//     if (Serial.available() > 0) {
//       String command = Serial.readStringUntil('\n');
//       command.trim();
      
//       if (command.equals("r")) {
//         encoder_reset();
//         Serial.println("Encoder reset");
//       } else {
//         pid_handle_command(command);
//       }
//     }
    
//     static unsigned long last_display_time = 0;
//     if (millis() - last_display_time >= 1000) {
//       Serial.print("RPM: ");
//       Serial.print(pid_get_current_rpm());
//       Serial.print(" | Target: ");
//       Serial.print(pid_get_target());
//       Serial.print(" | PWM: ");
//       Serial.println(motor_get_current_speed());
//       last_display_time = millis();
//     }
    
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("Starting ESP32-SAT...");

//   // lora_init();
//     // bmp180_init();
//   // mpu6050_init();
//   // init_gps();
//     // xTaskCreate(task_send_data_to_ground_station, "Task send data", 2048, NULL, 4, NULL);
//   // xTaskCreate(task_send_rpm_to_ground_station, "Task send rpm", 2048, NULL, 3, NULL);
//   // xTaskCreate(task_receive_data_from_ground_station, "Task receive data", 3072, NULL, 1, NULL);

//   motor_init();
//   encoder_init();
//   pid_init();
//   xTaskCreate(pid_task, "Task PID", 2048, NULL, 2, NULL);
//   xTaskCreate(serial_task, "Serial Task", 2048, NULL, 2, NULL);
// }

// void loop() {



// }


const unsigned long TEST_DURATION = 1000; // Thời gian test cho mỗi mức PWM (ms)
const unsigned long SAMPLE_TIME = 50;     // Thời gian lấy mẫu (ms)

// Các mức PWM muốn test (thay đổi theo động cơ của bạn)
// Giá trị từ 0-255, nên test cả chiều thuận và nghịch
int pwmLevels[] = { 150, 200, 250 , 300, 350, 400, 450, 500 , 550, 600, 650, 700, 750, 800}; 
int currentLevelIndex = 0;
unsigned long testStartTime = 0;
unsigned long lastSampleTime = 0;

// --- Thiết lập ---
void setup() {
  Serial.begin(115200); // Mở Serial với tốc độ cao
  // Cấu hình chân motor (đã có ở file khác nên có thể bỏ qua)
  // pinMode(MOTOR_ENA, OUTPUT);

  motor_init();
  encoder_init();
  
  Serial.println("Time_ms,PWM_Input,RPM_Output"); // Header cho CSV
  delay(2000); // Chờ ổn định
  testStartTime = millis();
}

// --- Vòng lặp chính ---
void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTestTime = currentTime - testStartTime;
  
  // Kiểm tra nếu đã hết thời gian test cho mức PWM hiện tại
  if (elapsedTestTime > TEST_DURATION) {
    currentLevelIndex++;
    testStartTime = currentTime;
    
    // Nếu đã test hết tất cả các mức, dừng chương trình
    if (currentLevelIndex >= sizeof(pwmLevels)/sizeof(pwmLevels[0])) {
      motor_set_speed(0); // Dừng động cơ
      Serial.println("Test completed!");
      while(1); // Dừng vĩnh viễn
    }
  }
  
  // Kiểm tra đến thời điểm lấy mẫu
  if (currentTime - lastSampleTime >= SAMPLE_TIME) {
    int currentPWM = pwmLevels[currentLevelIndex];
    
    // Áp dụng giá trị PWM cho động cơ
    motor_set_speed(currentPWM);
    // Đọc và tính toán RPM (hàm đã có sẵn)
    float currentRPM = encoder_get_rpm();
    // Xuất dữ liệu dạng CSV
    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(currentPWM);
    Serial.print(",");
    Serial.println(currentRPM, 4); // In RPM với 4 số thập phân
    
    lastSampleTime = currentTime;
  }
}
