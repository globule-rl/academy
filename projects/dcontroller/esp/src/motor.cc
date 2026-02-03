#include <motor/motor.h>


// fast ram storage, pulse cnt, isr sensor interrupt handler
void IRAM_ATTR hallM1ISR() {
  hallCountM1++;
  lastHallTime = millis();
}

void IRAM_ATTR hallM2ISR() {
  hallCountM2++;
  lastHallTime = millis();
}
void IRAM_ATTR motor1_hall_isr() {
    hall_counters[0]++;
}

void IRAM_ATTR motor2_hall_isr() {
    hall_counters[1]++;
}
void set_motor_speed(uint8_t motor_id, int speed) {
    if (motor_id > 1 || speed < -255 || speed > 255) return;
    
    // Clamp speed
    speed = constrain(speed, -255, 255);
    
    // Set direction
    uint8_t dir_pin = (motor_id == 0) ? MOTOR1_DIR_PIN : MOTOR2_DIR_PIN;
    gpio_set_level(dir_pin, speed >= 0 ? HIGH : LOW);
    
    // Set PWM
    uint8_t pwm_channel = motor_id;
    uint8_t pwm_value = abs(speed);
    ledcWrite(pwm_channel, pwm_value);
    
    xSemaphoreTake(motorStateMutex, portMAX_DELAY);
    motor_states[motor_id].current_speed = speed;
    xSemaphoreGive(motorStateMutex);
}

float read_current(uint8_t motor_id) {
    uint16_t adc_pin = (motor_id == 0) ? CURRENT_SENSOR1_PIN : CURRENT_SENSOR2_PIN;
    
    // Read multiple samples for averaging
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += adc_oneshot_read(adc_pin);
        delayMicroseconds(10);
    }
    uint16_t adc_value = sum / 10;
    
    // Convert ADC to voltage (3.3V / 4095)
    float voltage_mv = (adc_value * 3300) / 4095.0;
    
    // ACS712-5A: 2.5V offset, 185mV per A
    float offset_mv = 1650;  // 2.5V reference at half range
    float current_ma = (voltage_mv - offset_mv) / 0.185;
    
    return current_ma;
}

void check_over_current() {
    for (int i = 0; i < 2; i++) {
        float current = read_current(i);
        
        xSemaphoreTake(motorStateMutex, portMAX_DELAY);
        motor_states[i].current_ma = current;
        
        if (abs(current) > CURRENT_LIMIT_MA) {
            motor_states[i].over_current = true;
            set_motor_speed(i, 0);  // Stop motor
            Serial.printf("Motor %d: OVER CURRENT %.0f mA\n", i, current);
        } else {
            motor_states[i].over_current = false;
        }
        xSemaphoreGive(motorStateMutex);
    }
}

void task_command_handler(void *param) {
    const TickType_t xTicksToWait = pdMS_TO_TICKS(2000);  // Demo: command every 2s
    
    while (1) {
        vTaskDelay(xTicksToWait);
        
        // Demo commands - replace with actual input handling
        static int demo_state = 0;
        MotorCommand cmd;
        
        switch (demo_state % 4) {
            case 0:
                cmd = {100, 0};  // Motor 0: 100
                break;
            case 1:
                cmd = {150, 1};  // Motor 1: 150
                break;
            case 2:
                cmd = {-100, 0};  // Motor 0: reverse
                break;
            case 3:
                cmd = {0, 1};  // Motor 1: stop
                break;
        }
        
        xQueueSend(motorCommandQueue, &cmd, 0);
        demo_state++;
    }
}

void updateHeight() {
  long avgPulses = (hallCountM1 + hallCountM2) / 2;
  heightMM = avgPulses / PULSES_PER_MM;

  if (heightMM > MAX_HEIGHT_MM) {
    heightMM = MAX_HEIGHT_MM;
    stopMotors();
  }
  if (heightMM < MIN_HEIGHT_MM) {
    heightMM = MIN_HEIGHT_MM;
    stopMotors();
  }
}

float readCurrent(int pin) {
  int raw = adc_oneshot_read(pin);
  float voltage = (raw / 4095.0) * 3.3;
  return (voltage - 2.5) / 0.066; // ACS712 30A example
}

void safetyCheck() {
  float i1 = readCurrent(CUR_M1);
  float i2 = readCurrent(CUR_M2);

  if (abs(i1) > CURRENT_LIMIT || abs(i2) > CURRENT_LIMIT) {
    stopMotors();
  }

  if (motion != STOPPED && millis() - lastHallTime > STALL_TIMEOUT) {
    stopMotors();
  }
}

void task_motor_control(void *param) {
    const TickType_t xTicksToWait = pdMS_TO_TICKS(50);  // 50ms update rate
    
    while (1) {
        MotorCommand cmd;
        
        if (xQueueReceive(motorCommandQueue, &cmd, 0) == pdPASS) {
            xSemaphoreTake(motorStateMutex, portMAX_DELAY);
            motor_states[cmd.motor_id].target_speed = cmd.speed;
            xSemaphoreGive(motorStateMutex);
        }
        
        // Implement smooth acceleration (max 10 units per update)
        for (int i = 0; i < 2; i++) {
            xSemaphoreTake(motorStateMutex, portMAX_DELAY);
            int current = motor_states[i].current_speed;
            int target = motor_states[i].target_speed;
            xSemaphoreGive(motorStateMutex);
            
            if (current < target) {
                set_motor_speed(i, min(current + 10, target));
            } else if (current > target) {
                set_motor_speed(i, max(current - 10, target));
            }
        }
        
        vTaskDelay(xTicksToWait);
    }
}

// Task: Current Monitoring
void task_current_monitor(void *param) {
    const TickType_t xTicksToWait = pdMS_TO_TICKS(100);  // 100ms check rate
    
    while (1) {
        check_over_current();
        vTaskDelay(xTicksToWait);
    }
}

// Task: Hall Sensor Tracking (RPM calculation)
void task_hall_monitor(void *param) {
    const TickType_t xTicksToWait = pdMS_TO_TICKS(1000);  // 1 second
    int prev_counts[2] = {0, 0};
    
    while (1) {
        vTaskDelay(xTicksToWait);
        
        for (int i = 0; i < 2; i++) {
            int delta = hall_counters[i] - prev_counts[i];
            float rpm = (delta / 6.0) * 60;  // Assuming 6 Hall events per revolution
            
            xSemaphoreTake(motorStateMutex, portMAX_DELAY);
            motor_states[i].hall_count = hall_counters[i];
            xSemaphoreGive(motorStateMutex);
            
            Serial.printf("Motor %d: %d edges/sec, ~%.1f RPM\n", i, delta, rpm);
        }
        
        prev_counts[0] = hall_counters[0];
        prev_counts[1] = hall_counters[1];
    }
}

// Task: Telemetry / Serial Logging
void task_telemetry(void *param) {
    const TickType_t xTicksToWait = pdMS_TO_TICKS(500);
    
    while (1) {
        vTaskDelay(xTicksToWait);
        
        xSemaphoreTake(motorStateMutex, portMAX_DELAY);
        Serial.println("=== MOTOR STATUS ===");
        for (int i = 0; i < 2; i++) {
            Serial.printf("Motor %d | Speed: %d | Current: %.0f mA | Over-Current: %s\n",
                         i, 
                         motor_states[i].current_speed,
                         motor_states[i].current_ma,
                         motor_states[i].over_current ? "YES" : "NO");
        }
        xSemaphoreGive(motorStateMutex);
    }
}