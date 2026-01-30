#include <freertos/idf_additions.h>
#include <ostream>
#include <pin.h>
#include "esp_wifi.h"


void taskBlink(void *parameter) {
    // led
}
void taskSerial(void *parameter) {
    // display
}
void setup() {
  Serial.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_M1, INPUT_PULLUP);
  pinMode(BTN_M2, INPUT_PULLUP);

  ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_IN1, PWM_CH_M1);
  ledcAttachPin(M2_IN1, PWM_CH_M2);

  attachInterrupt(HALL_M1, hallM1ISR, RISING);
  attachInterrupt(HALL_M2, hallM2ISR, RISING);

  prefs.begin(PREF_NAMESPACE, false);

  Wire.begin(SDA_PIN, SCL_PIN);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

void loop() {
  handleButtons();
  updateHeight();
  safetyCheck();
  updateDisplay();
  delay(50);
}

void app_main(void)
{ 
    xTaskCreatePinnedToCore(
    taskBlink,           // Function to call
    "BlinkTask",         // Task name
    2048,                // Stack size (bytes)
    NULL,                // Parameters
    1,                   // Priority
    NULL,                // Task handle
    1                    // Core (0 or 1)
  );
  
  // Create Task 2: Serial (Priority 1, Core 0)
  xTaskCreatePinnedToCore(
    taskSerial,          // Function to call
    "SerialTask",        // Task name
    2048,                // Stack size (bytes)
    NULL,                // Parameters
    1,                   // Priority
    NULL,                // Task handle
    0                    // Core (0 or 1)
  );

}