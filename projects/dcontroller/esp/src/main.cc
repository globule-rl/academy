#include <ostream>
#include <motor/pin.h>
#include <motor/motor.h>
#include <motor/display.h>

void init_pwm() {
    // Configure PWM channels
    ledc_timer_config(0, PWM_FREQ, PWM_RESOLUTION);  // Motor 1
    ledc_timer_config(1, PWM_FREQ, PWM_RESOLUTION);  // Motor 2
    
    ledcAttachPin(MOTOR1_PWM_PIN, 0);
    ledcAttachPin(MOTOR2_PWM_PIN, 1);
}

void init_motors() {
    // Direction pins
    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    
    gpio_set_level(MOTOR1_DIR_PIN, LOW);
    gpio_set_level(MOTOR2_DIR_PIN, LOW);
}

void init_hall_sensors() {
    pinMode(MOTOR1_HALL_A, INPUT_PULLUP);
    pinMode(MOTOR1_HALL_B, INPUT_PULLUP);
    pinMode(MOTOR2_HALL_A, INPUT_PULLUP);
    pinMode(MOTOR2_HALL_B, INPUT_PULLUP);
    
    // Attach interrupts on HALL_A pins
    gpio_isr_handler_add(digitalPinToInterrupt(MOTOR1_HALL_A), motor1_hall_isr, RISING);
    gpio_isr_handler_add(digitalPinToInterrupt(MOTOR2_HALL_A), motor2_hall_isr, RISING);
}

void init_current_sensors() {
    pinMode(CURRENT_SENSOR1_PIN, INPUT);
    pinMode(CURRENT_SENSOR2_PIN, INPUT);
    
    analogSetAttenuation(ADC_11db);  // Full ADC range
    adc_oneshot_readResolution(12);  // 12-bit resolution
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== ESP32 CONTROLLER ===\n");
  
  // Initialize hardware
  init_pwm();
  init_motors();
  init_hall_sensors();
  init_current_sensors();
  
  // Create synchronization primitives
  motorCommandQueue = xQueueCreate(10, sizeof(MotorCommand));
  motorStateMutex = xSemaphoreCreateMutex();
  

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_M1, INPUT_PULLUP);
  pinMode(BTN_M2, INPUT_PULLUP);

  ledc_timer_config(PWM_CH_M1, PWM_FREQ, PWM_RES);
  ledc_timer_config(PWM_CH_M2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_IN1, PWM_CH_M1);
  ledcAttachPin(M2_IN1, PWM_CH_M2);

  gpio_isr_handler_add(HALL_M1, hallM1ISR, RISING);
  gpio_isr_handler_add(HALL_M2, hallM2ISR, RISING);

  prefs.begin(PREF_NAMESPACE, false);

  Wire.begin(SDA_PIN, SCL_PIN);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}


void app_main(void)
{ 
    xTaskCreatePinnedToCore(
    task_motor_control,   // Function to call
    "Motor",              // Task name
    2048,                // Stack size (bytes)
    NULL,                // Parameters
    3,                   // Priority
    NULL,                // Task handle
    0                    // Core (0 or 1)
  );
 xTaskCreatePinnedToCore(
    task_current_monitor, "CurrentMon", 2048, NULL, 3, NULL, 0);
 xTaskCreatePinnedToCore(
    task_hall_monitor, "HallMon", 2048, NULL, 2, NULL, 1);
 xTaskCreatePinnedToCore(
    task_command_handler, "CmdHandler", 2048, NULL, 2, NULL, 1);
 xTaskCreatePinnedToCore(
    task_telemetry, "Telemetry", 2048, NULL, 1, NULL, 1);

  xTaskCreatePinnedToCore(
    task_display, "Diplay", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(
    task_webserver, "WebServer", 2048, NULL, 1, NULL, 1);
}

void loop() {
  updateDisplay();
      vTaskDelay(pdMS_TO_TICKS(10000));  // Idle task
}