#include "wifi_log.h"

// Biến toàn cục
WebServer server(80);  // Web server trên port 80
String logData = "";   // Lưu trữ log data

void handleClientTask(void *parameter) {
    while (true) {
        server.handleClient();
        delay(10);
    }
}

void wifi_connect() {
    const char* ssid = "Vnpt_Duy Thai";
    const char* password = "namthang";
        
    // Kết nối WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    // Chờ kết nối
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Thiết lập web server
    server.on("/", []() {
        String html = "<html><head><title>ESP32 Log</title>";
        html += "<meta http-equiv='refresh' content='0.5'>"; // Tự động refresh 2 giây
        html += "</head><body>";
        html += "<h1>ESP32 Log Viewer</h1>";
        html += "<div style='border:1px solid #ccc; padding:10px; background:#f9f9f9;'>";
        html += "<pre>" + logData + "</pre>";
        html += "</div>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    });
    
    server.on("/clear", []() {
        logData = "";
        server.send(200, "text/plain", "Log cleared");
    });
    
    server.begin();
    xTaskCreate(handleClientTask, "HTTP_Server", 4096, NULL, 1, NULL);
    Serial.println("HTTP server started");
    
    // Thêm log ban đầu
    wifi_log("System started");
    wifi_log("WiFi connected successfully");
}

void wifi_log(const char* message) {
    // Thêm timestamp
    unsigned long currentTime = millis();
    String timestamp = String(currentTime / 1000) + "s";
    
    // Thêm vào log data
    logData += "[" + timestamp + "] " + String(message) + "\n";
    
    // Giới hạn độ dài log (giữ 50 dòng gần nhất)
    int lineCount = 0;
    for (int i = 0; i < logData.length(); i++) {
        if (logData.charAt(i) == '\n') lineCount++;
    }
    
    if (lineCount > 50) {
        int firstNewline = logData.indexOf('\n');
        if (firstNewline != -1) {
            logData = logData.substring(firstNewline + 1);
        }
    }
    
    // In ra Serial
    Serial.println(message);
    
}

void wifi_logf(const char* format, ...) {
    char buffer[256];  // Buffer để lưu chuỗi định dạng
    
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Sử dụng hàm wifi_log có sẵn
    wifi_log(buffer);
}