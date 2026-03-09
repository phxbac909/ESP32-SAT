# 🚁 UAV Edge Nodes: Telemetry & Video Streaming System

Repository này chứa mã nguồn nhúng (Firmware) cho cụm thiết bị thu thập dữ liệu và hình ảnh gắn trên Quadcopter. Hệ thống bao gồm 2 vi điều khiển hoạt động song song: một **ESP32** chuyên xử lý cảm biến và truyền tin xa qua LoRa, và một **ESP32-CAM** chuyên truyền phát video thời gian thực.


---

## 🌟 Tính năng chính (Features)

* **Thu thập Telemetry:** Lấy dữ liệu tư thế (MPU6050), độ cao/áp suất (BMP280) và tọa độ (GPS Neo-6M).
* **Truyền tin NLOS (Non-Line-of-Sight):** Đóng gói dữ liệu và gửi về trạm mặt đất qua sóng LoRa (module RA-01H) để đảm bảo khoảng cách truyền xa và ổn định.
* **Low-Latency Video Stream:** Sử dụng ESP32-CAM capture hình ảnh và stream qua giao thức UDP (WiFi) về Ground Station để phục vụ xử lý AI.
* **Thiết kế phân tán:** Tách biệt tác vụ xử lý video và tác vụ xử lý cảm biến/điều khiển bay để tránh nghẽn cổ chai CPU.

---

## 🛠 Sơ đồ kết nối phần cứng (Wiring Diagram)



### 1. Cụm Telemetry (Main ESP32)

| Module Cảm biến | Giao tiếp | Chân ESP32 (Gợi ý) | Chức năng |
| :--- | :--- | :--- | :--- |
| **MPU6050** | I2C | SDA (21), SCL (22) | Góc nghiêng (Roll, Pitch, Yaw) |
| **BMP280** | I2C | SDA (21), SCL (22) | Áp suất, nhiệt độ, độ cao |
| **GPS Neo-6M** | UART | RX (16), TX (17) | Tọa độ Kinh/Vĩ độ, Vận tốc |
| **LoRa RA-01H** | SPI | SCK(18), MISO(19), MOSI(23), CS(5) | Truyền gói tin Telemetry |


### 2. Cụm Video Stream (ESP32-CAM)

* Cấp nguồn ổn định 5V/2A để tránh hiện tượng sụt áp khi khởi động module Camera (OV2640).
* Kết nối Antenna ngoài (nếu có) để tăng tầm phát sóng WiFi UDP.

---

## 📦 Định dạng gói dữ liệu (Payload Structure)

Dữ liệu cảm biến từ Main ESP32 được đóng gói thành một `struct`  trước khi gửi qua LoRa để tối ưu hóa băng thông (Payload Size).
