#ifndef WIFI_LOG_H
#define WIFI_LOG_H

#include <WiFi.h>
#include <WebServer.h>

// Khai báo các hàm
void wifi_connect();
void wifi_log(const char* message);
void wifi_logf(const char* format, ...);

#endif