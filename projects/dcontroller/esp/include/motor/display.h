#pragma once

#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_wifi.h"

Preferences prefs;
// Credentials - CHANGE THESE!
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

WebServer server(80);

// Extern declarations from main.cpp
extern QueueHandle_t motorCommandQueue;
extern MotorState motor_states[2];
extern SemaphoreHandle_t motorStateMutex;

void task_display(void *param) {}
void task_webserver(void *param) {}
