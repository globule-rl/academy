#include <motor/display.h>

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Height:");
  display.setCursor(0, 30);
  display.print(heightMM, 1);
  display.print("mm");
  display.display();
}

void handleButtons() {
  if (!digitalRead(BTN_UP)) moveUp();
  else if (!digitalRead(BTN_DOWN)) moveDown();
  else stopMotors();

  if (!digitalRead(BTN_M1)) {
    prefs.putFloat(PREF_H1, heightMM);
    delay(300);
  }

  if (!digitalRead(BTN_M2)) {
    prefs.putFloat(PREF_H2, heightMM);
    delay(300);
  }
}

void task_display(void *param) {
    handleButtons();
    updateDisplay();
}


// ============ WEB HANDLERS ============
void handleMotorControl() {
    if (server.method() == HTTP_POST) {
        String body = server.arg("plain");
        StaticJsonDocument<200> doc;
        
        if (deserializeJson(doc, body) == DeserializationError::Ok) {
            uint8_t motor_id = doc["motor"] | 0;
            int speed = doc["speed"] | 0;
            
            MotorCommand cmd = {speed, motor_id};
            xQueueSend(motorCommandQueue, &cmd, 0);
            
            server.send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        }
    } else {
        server.send(405, "text/plain", "Method not allowed");
    }
}

void handleGetStatus() {
    StaticJsonDocument<300> doc;
    
    xSemaphoreTake(motorStateMutex, portMAX_DELAY);
    doc["motor0"]["speed"] = motor_states[0].current_speed;
    doc["motor0"]["current_ma"] = (int)motor_states[0].current_ma;
    doc["motor0"]["over_current"] = motor_states[0].over_current;
    
    doc["motor1"]["speed"] = motor_states[1].current_speed;
    doc["motor1"]["current_ma"] = (int)motor_states[1].current_ma;
    doc["motor1"]["over_current"] = motor_states[1].over_current;
    xSemaphoreGive(motorStateMutex);
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleRoot() {
    String html = R"(
    <!DOCTYPE html>
    <html>
    <head>
        <title>d Controller</title>
        <style>
            body { font-family: Arial; margin: 20px; }
            .motor { border: 1px solid #ccc; padding: 20px; margin: 10px 0; }
            button { padding: 10px 20px; margin: 5px; font-size: 16px; }
            .status { background: #eee; padding: 10px; margin: 10px 0; }
        </style>
    </head>
    <body>
        <h1>ESP32 d Controller</h1>
        
        <div class="motor">
            <h2>Motor 0 (Up)</h2>
            <button onclick="sendCmd(0, 100)">Forward</button>
            <button onclick="sendCmd(0, 0)">Stop</button>
            <button onclick="sendCmd(0, -100)">Reverse</button>
            <div class="status" id="status0"></div>
        </div>
        
        <div class="motor">
            <h2>Motor 1 (Down)</h2>
            <button onclick="sendCmd(1, 100)">Forward</button>
            <button onclick="sendCmd(1, 0)">Stop</button>
            <button onclick="sendCmd(1, -100)">Reverse</button>
            <div class="status" id="status1"></div>
        </div>
        
        <script>
            function sendCmd(motor, speed) {
                fetch('/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({motor: motor, speed: speed})
                });
            }
            
            function updateStatus() {
                fetch('/status').then(r => r.json()).then(data => {
                    document.getElementById('status0').innerHTML = 
                        'Speed: ' + data.motor0.speed + 
                        ' | Current: ' + data.motor0.current_ma + 'mA' +
                        ' | OC: ' + (data.motor0.over_current ? 'YES' : 'NO');
                    document.getElementById('status1').innerHTML = 
                        'Speed: ' + data.motor1.speed + 
                        ' | Current: ' + data.motor1.current_ma + 'mA' +
                        ' | OC: ' + (data.motor1.over_current ? 'YES' : 'NO');
                });
            }
            
            setInterval(updateStatus, 500);
            updateStatus();
        </script>
    </body>
    </html>
    )";
    server.send(200, "text/html", html);
}

void task_webserver(void *param) {
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected to WiFi. IP: ");
        Serial.println(WiFi.localIP());
        
        server.on("/", HTTP_GET, handleRoot);
        server.on("/control", HTTP_POST, handleMotorControl);
        server.on("/status", HTTP_GET, handleGetStatus);
        server.begin();
        
        while (1) {
            server.handleClient();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        Serial.println("Failed to connect to WiFi");
        vTaskDelete(NULL);
    }
}

