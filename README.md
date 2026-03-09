# ESP32 Drone Flight Controller (PID Dual-Loop & ESP-NOW)

Dự án này là một bộ điều khiển bay (Flight Controller) tùy chỉnh cho Drone (Quadcopter) dựa trên vi điều khiển **ESP32**. Hệ thống sử dụng thuật toán **PID kép (Cascade PID)** để kiểm soát thái độ bay, giao tiếp không dây tốc độ cao qua **ESP-NOW** và tích hợp cơ chế **Failsafe** đa tầng.

## 🚀 Tính năng nổi bật

* **Điều khiển không dây (ESP-NOW):** Sử dụng giao thức ESP-NOW cho độ trễ cực thấp (< 5ms), hỗ trợ Telemetry gửi dữ liệu ngược về bộ điều khiển.
* **PID Dual-Loop (Cascade):** * *Vòng ngoài (Outer Loop):* Điều khiển góc (Euler Angle).
    * *Vòng trong (Inner Loop):* Điều khiển tốc độ góc (Angular Rate/Gyro) chạy ở tần số **250Hz** ($dt = 4ms$).
* **Xử lý đa nhân (Multicore):** * **Core 0:** Xử lý kết nối ESP-NOW và giám sát tác vụ Failsafe.
    * **Core 1:** Dành riêng cho tính toán PID và cập nhật Motor để đảm bảo tính thời gian thực tuyệt đối.
* **Failsafe thông minh:** Tự động hạ dần ga (Throttle) và đưa các góc cân bằng về $0^\circ$ khi mất tín hiệu quá 1 giây.
* **Bộ lọc bù (Complementary Filter):** Kết hợp dữ liệu Gyroscope và Accelerometer từ MPU6050 với hằng số thời gian $\tau = 10.0$ để ước lượng góc nghiêng chính xác, chống trôi (drift).



---

## 🛠 Sơ đồ kết nối (Tham khảo)

| Linh kiện | Chân ESP32 | Ghi chú |
|---|---|---|
| **MPU6050 SDA** | GPIO 21 | I2C Data |
| **MPU6050 SCL** | GPIO 22 | I2C Clock |
| **Motor 1** | (Config.h) | PWM Output (Front-Right) |
| **Motor 2** | (Config.h) | PWM Output (Front-Left) |
| **Motor 3** | (Config.h) | PWM Output (Back-Left) |
| **Motor 4** | (Config.h) | PWM Output (Back-Right) |



---

## ⚙️ Cấu hình hệ thống

### 1. Thông số PID mặc định
Hệ thống sử dụng bộ thông số đã được tinh chỉnh (Tuned) để giảm rung lắc:
* **Angle Loop:** $K_p = 2.0, K_i = 0.0, K_d = 0.0$
* **Rate Loop:** $K_p = 0.8, K_i = 0.0, K_d = 0.0003$

### 2. Định dạng dữ liệu (Data Struct)
Hệ thống nhận dạng gói tin qua `ID` byte đầu tiên:
* `ID = 1`: Dữ liệu điều khiển (Speed, Roll, Pitch, Yaw).
* `ID = 2`: Cập nhật thông số PID từ xa.
* `ID = 0`: Ngắt kết nối motor khẩn cấp.

---

## ⚠️ Cơ chế An toàn (Safety)

1.  **Góc nghiêng giới hạn:** Nếu Drone nghiêng quá **$37^\circ$** (do va chạm hoặc lật), hệ thống sẽ ngắt Motor ngay lập tức để bảo vệ linh kiện.
2.  **Mất sóng (Failsafe Task):** Nếu không nhận được dữ liệu từ Remote > 1000ms:
    * Tự động đưa `Roll`, `Pitch`, `Yaw` về mức cân bằng ($0^\circ$).
    * Giảm dần `Throttle` (mỗi 500ms giảm 40 đơn vị) để hạ cánh an toàn thay vì rơi tự do.

---

## 📂 Cấu trúc mã nguồn

* `esp32_now.cpp`: Khởi tạo ESP-NOW, đăng ký Peer và xử lý callback nhận dữ liệu, tạo task chạy ngầm trên Core 0 để giám sát kết nối.
* `pid_euler_250Hz.cpp`: Triển khai vòng lặp PID 250Hz trên Core 1 và Motor Mixing.
* `mpu6050.cpp`: Cấu hình thanh ghi MPU6050, tính toán bù nhiễu (Offset) và lọc dữ liệu cảm biến.
: 

---


**LƯU Ý:** Việc bay Drone có thể gây nguy hiểm. Luôn tháo cánh quạt khi thực hiện nạp code, thử nghiệm PID hoặc cấu hình Failsafe trên bàn làm việc.
