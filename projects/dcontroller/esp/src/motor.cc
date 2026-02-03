#include "motor/motor.h"
#include "motor/pin.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "motor";

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Hall sensor pulse counters (ISR-safe)
volatile int32_t hall_counters[MOTOR_COUNT] = {0, 0};
volatile uint32_t last_hall_time_ms[MOTOR_COUNT] = {0, 0};

// Legacy variables (for compatibility)
volatile long hallCountM1 = 0;
volatile long hallCountM2 = 0;
float heightMM = 0;
unsigned long lastHallTime = 0;

// System state
desk_state_t desk_state = {
    .height_mm = 0.0f,
    .target_height_mm = 0.0f,
    .mem1_height = -1.0f,
    .mem2_height = -1.0f,
    .calibrated = false
};

motion_state_t motion_state = MOTION_STOPPED;

// Motor states (protected by mutex)
motor_state_t motor_states[MOTOR_COUNT] = {
    {0, 0, 0.0f, 0, false, false, false, false, false, 25.0f},
    {0, 0, 0.0f, 0, false, false, false, false, false, 25.0f}
};

// FreeRTOS primitives
QueueHandle_t motor_command_queue = NULL;
SemaphoreHandle_t motor_state_mutex = NULL;
SemaphoreHandle_t desk_state_mutex = NULL;

// ADC calibration
esp_adc_cal_characteristics_t adc_cal_chars;

// ============================================================================
// ISR HANDLERS
// ============================================================================

void IRAM_ATTR motor1_hall_isr(void* arg) {
    hall_counters[MOTOR_ID_1]++;
    hallCountM1 = hall_counters[MOTOR_ID_1];
    last_hall_time_ms[MOTOR_ID_1] = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void IRAM_ATTR motor2_hall_isr(void* arg) {
    hall_counters[MOTOR_ID_2]++;
    hallCountM2 = hall_counters[MOTOR_ID_2];
    last_hall_time_ms[MOTOR_ID_2] = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void IRAM_ATTR limit_switch_isr(void* arg) {
    // Emergency stop on limit switch trigger
    gpio_num_t gpio = (gpio_num_t)(intptr_t)arg;
    
    if (gpio == LIMIT_TOP_M1 || gpio == LIMIT_BOT_M1) {
        motor_states[MOTOR_ID_1].limit_triggered = true;
        if (motor_states[MOTOR_ID_1].current_speed != 0) {
            set_motor_speed(MOTOR_ID_1, 0);
        }
    }
    if (gpio == LIMIT_TOP_M2 || gpio == LIMIT_BOT_M2) {
        motor_states[MOTOR_ID_2].limit_triggered = true;
        if (motor_states[MOTOR_ID_2].current_speed != 0) {
            set_motor_speed(MOTOR_ID_2, 0);
        }
    }
}

void IRAM_ATTR red_wire_isr(void* arg) {
    gpio_num_t gpio = (gpio_num_t)(intptr_t)arg;
    
    if (gpio == M1_RED_PIN) {
        motor_states[MOTOR_ID_1].red_wire_alarm = true;
        // Only trigger shutdown if configured as thermal cutoff
        if (RED_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) {
            motor_states[MOTOR_ID_1].thermal_shutdown = true;
            set_motor_speed(MOTOR_ID_1, 0);
        }
    }
    if (gpio == M2_RED_PIN) {
        motor_states[MOTOR_ID_2].red_wire_alarm = true;
        if (RED_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) {
            motor_states[MOTOR_ID_2].thermal_shutdown = true;
            set_motor_speed(MOTOR_ID_2, 0);
        }
    }
}

void IRAM_ATTR yellow_wire_isr(void* arg) {
    gpio_num_t gpio = (gpio_num_t)(intptr_t)arg;
    
    if (gpio == M1_YELLOW_PIN) {
        motor_states[MOTOR_ID_1].yellow_wire_alarm = true;
        // Yellow wire behavior depends on configuration
        if (YELLOW_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) {
            motor_states[MOTOR_ID_1].thermal_shutdown = true;
            set_motor_speed(MOTOR_ID_1, 0);
        }
    }
    if (gpio == M2_YELLOW_PIN) {
        motor_states[MOTOR_ID_2].yellow_wire_alarm = true;
        if (YELLOW_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) {
            motor_states[MOTOR_ID_2].thermal_shutdown = true;
            set_motor_speed(MOTOR_ID_2, 0);
        }
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void motor_init(void) {
    ESP_LOGI(TAG, "Initializing motor hardware");
    
    // Configure PWM timer
    ledc_timer_config_t pwm_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));
    
    // Configure Motor 1 PWM channel
    ledc_channel_config_t m1_pwm = {
        .gpio_num = M1_IN1,
        .speed_mode = PWM_MODE,
        .channel = M1_PWM_CH,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m1_pwm));
    
    // Configure Motor 2 PWM channel
    ledc_channel_config_t m2_pwm = {
        .gpio_num = M2_IN1,
        .speed_mode = PWM_MODE,
        .channel = M2_PWM_CH,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m2_pwm));
    
    // Configure direction pins as outputs
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << M1_IN2) | (1ULL << M2_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&dir_conf));
    
    // Set initial direction to stopped
    gpio_set_level(M1_IN2, 0);
    gpio_set_level(M2_IN2, 0);
    
    ESP_LOGI(TAG, "Motor hardware initialized");
}

void sensors_init(void) {
    ESP_LOGI(TAG, "Initializing sensors");
    
    // Configure Hall sensor pins as inputs with pull-up
    gpio_config_t hall_conf = {
        .pin_bit_mask = (1ULL << HALL_M1) | (1ULL << HALL_M2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&hall_conf));
    
    // Install GPIO ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // Attach Hall sensor ISRs
    ESP_ERROR_CHECK(gpio_isr_handler_add(HALL_M1, motor1_hall_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HALL_M2, motor2_hall_isr, NULL));
    
    // Configure limit switch pins as inputs with pull-up (active low)
    gpio_config_t limit_conf = {
        .pin_bit_mask = (1ULL << LIMIT_TOP_M1) | (1ULL << LIMIT_BOT_M1) |
                       (1ULL << LIMIT_TOP_M2) | (1ULL << LIMIT_BOT_M2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Trigger on falling edge (active low)
    };
    ESP_ERROR_CHECK(gpio_config(&limit_conf));
    
    // Attach limit switch ISRs
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_TOP_M1, limit_switch_isr, (void*)LIMIT_TOP_M1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_BOT_M1, limit_switch_isr, (void*)LIMIT_BOT_M1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_TOP_M2, limit_switch_isr, (void*)LIMIT_TOP_M2));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_BOT_M2, limit_switch_isr, (void*)LIMIT_BOT_M2));
    
    // Configure motor end red wire pins as inputs with pull-up (active low alarm)
    gpio_config_t red_wire_conf = {
        .pin_bit_mask = (1ULL << M1_RED_PIN) | (1ULL << M2_RED_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Trigger on falling edge (alarm active)
    };
    ESP_ERROR_CHECK(gpio_config(&red_wire_conf));
    
    // Configure motor end yellow wire pins as inputs with pull-up
    gpio_config_t yellow_wire_conf = {
        .pin_bit_mask = (1ULL << M1_YELLOW_PIN) | (1ULL << M2_YELLOW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Trigger on falling edge (alarm active)
    };
    ESP_ERROR_CHECK(gpio_config(&yellow_wire_conf));
    
    // Attach red wire ISRs (primary thermal/overtemp)
    ESP_ERROR_CHECK(gpio_isr_handler_add(M1_RED_PIN, red_wire_isr, (void*)M1_RED_PIN));
    ESP_ERROR_CHECK(gpio_isr_handler_add(M2_RED_PIN, red_wire_isr, (void*)M2_RED_PIN));
    
    // Attach yellow wire ISRs (secondary alarm)
    ESP_ERROR_CHECK(gpio_isr_handler_add(M1_YELLOW_PIN, yellow_wire_isr, (void*)M1_YELLOW_PIN));
    ESP_ERROR_CHECK(gpio_isr_handler_add(M2_YELLOW_PIN, yellow_wire_isr, (void*)M2_YELLOW_PIN));
    
    // Initialize ADC
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(CUR_M1, ADC_ATTEN);
    adc1_config_channel_atten(CUR_M2, ADC_ATTEN);
    
    // Characterize ADC
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, ADC_VREF, &adc_cal_chars);
    
    ESP_LOGI(TAG, "Sensors initialized");
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void set_motor_speed(motor_id_t motor_id, int16_t speed) {
    if (motor_id >= MOTOR_COUNT) return;
    
    // Clamp speed to valid range
    speed = CLAMP(speed, -PWM_MAX, PWM_MAX);
    
    // Set direction
    gpio_num_t dir_pin = (motor_id == MOTOR_ID_1) ? M1_IN2 : M2_IN2;
    gpio_set_level(dir_pin, speed >= 0 ? 1 : 0);
    
    // Set PWM duty
    ledc_channel_t pwm_ch = (motor_id == MOTOR_ID_1) ? M1_PWM_CH : M2_PWM_CH;
    uint32_t duty = ABS(speed);
    ledc_set_duty(PWM_MODE, pwm_ch, duty);
    ledc_update_duty(PWM_MODE, pwm_ch);
    
    // Update state with mutex protection
    if (motor_state_mutex != NULL) {
        xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
    }
    motor_states[motor_id].current_speed = speed;
    if (motor_state_mutex != NULL) {
        xSemaphoreGive(motor_state_mutex);
    }
    
    ESP_LOGD(TAG, "Motor %d speed set to %d", motor_id, speed);
}

void stop_motors(void) {
    set_motor_speed(MOTOR_ID_1, 0);
    set_motor_speed(MOTOR_ID_2, 0);
    motion_state = MOTION_STOPPED;
    ESP_LOGI(TAG, "All motors stopped");
}

void move_up(void) {
    // Check top limits first
    if (check_limit_switch(MOTOR_ID_1, true) || check_limit_switch(MOTOR_ID_2, true)) {
        ESP_LOGW(TAG, "Cannot move up: top limit reached");
        return;
    }
    
    motion_state = MOTION_MOVING_UP;
    set_motor_speed(MOTOR_ID_1, MOTOR_SPEED);
    set_motor_speed(MOTOR_ID_2, MOTOR_SPEED);
    ESP_LOGI(TAG, "Moving up");
}

void move_down(void) {
    // Check bottom limits first
    if (check_limit_switch(MOTOR_ID_1, false) || check_limit_switch(MOTOR_ID_2, false)) {
        ESP_LOGW(TAG, "Cannot move down: bottom limit reached");
        return;
    }
    
    motion_state = MOTION_MOVING_DOWN;
    set_motor_speed(MOTOR_ID_1, -MOTOR_SPEED);
    set_motor_speed(MOTOR_ID_2, -MOTOR_SPEED);
    ESP_LOGI(TAG, "Moving down");
}

bool move_to_height(float target_height_mm, uint32_t timeout_ms) {
    if (!desk_state.calibrated) {
        ESP_LOGW(TAG, "Cannot move to height: not calibrated");
        return false;
    }
    
    float current = get_current_height();
    float tolerance = 2.0f;  // 2mm tolerance
    
    if (ABS(target_height_mm - current) <= tolerance) {
        return true;  // Already at target
    }
    
    TickType_t start_time = xTaskGetTickCount();
    
    if (target_height_mm > current) {
        move_up();
    } else {
        move_down();
    }
    
    // Wait until we reach target or timeout
    while ((xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS < timeout_ms) {
        current = get_current_height();
        
        if (ABS(target_height_mm - current) <= tolerance) {
            stop_motors();
            ESP_LOGI(TAG, "Reached target height: %.1f mm", current);
            return true;
        }
        
        // Check for limits or safety conditions
        if (motion_state == MOTION_MOVING_UP && 
            (check_limit_switch(MOTOR_ID_1, true) || check_limit_switch(MOTOR_ID_2, true))) {
            stop_motors();
            ESP_LOGW(TAG, "Stopped: top limit reached");
            return false;
        }
        
        if (motion_state == MOTION_MOVING_DOWN && 
            (check_limit_switch(MOTOR_ID_1, false) || check_limit_switch(MOTOR_ID_2, false))) {
            stop_motors();
            ESP_LOGW(TAG, "Stopped: bottom limit reached");
            return false;
        }
        
        safety_check();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    stop_motors();
    ESP_LOGW(TAG, "Move timeout");
    return false;
}

// ============================================================================
// SENSORS
// ============================================================================

float read_current(motor_id_t motor_id) {
    adc1_channel_t channel = (motor_id == MOTOR_ID_1) ? CUR_M1 : CUR_M2;
    
    // Read multiple samples and average
    uint32_t adc_sum = 0;
    for (int i = 0; i < 10; i++) {
        adc_sum += adc1_get_raw(channel);
        esp_rom_delay_us(100);
    }
    uint32_t adc_raw = adc_sum / 10;
    
    // Convert to voltage (mV)
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_raw, &adc_cal_chars);
    
    // ACS712-30A: 2.5V offset, 66mV/A sensitivity
    float offset_mv = ACS712_OFFSET;
    float current_a = (voltage_mv - offset_mv) / ACS712_SENS;
    
    return current_a * 1000.0f;  // Return in mA
}

float read_motor_end_sensors(motor_id_t motor_id, bool* red_wire, bool* yellow_wire) {
    gpio_num_t red_pin = (motor_id == MOTOR_ID_1) ? M1_RED_PIN : M2_RED_PIN;
    gpio_num_t yellow_pin = (motor_id == MOTOR_ID_1) ? M1_YELLOW_PIN : M2_YELLOW_PIN;
    
    // Read both pins (configurable active level)
    int red_level = gpio_get_level(red_pin);
    int yellow_level = gpio_get_level(yellow_pin);
    
    // Check if alarm is active based on configured logic
    *red_wire = (red_level == RED_WIRE_LOGIC);
    *yellow_wire = (yellow_level == YELLOW_WIRE_LOGIC);
    
    // Return estimated temperature based on wire function configuration
    // This is a placeholder - actual temperature calculation depends on sensor type
    if (*red_wire) {
        // Red wire alarm active - at or above threshold
        return (RED_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) ? THERMAL_CUTOFF_TEMP_C : THERMAL_WARNING_TEMP_C;
    } else if (*yellow_wire) {
        // Yellow wire alarm active - approaching threshold
        return (YELLOW_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) ? THERMAL_CUTOFF_TEMP_C : THERMAL_WARNING_TEMP_C;
    } else {
        // Both normal - assume nominal operating temperature
        return 25.0f;
    }
}

bool check_limit_switch(motor_id_t motor_id, bool top) {
    gpio_num_t pin;
    if (motor_id == MOTOR_ID_1) {
        pin = top ? LIMIT_TOP_M1 : LIMIT_BOT_M1;
    } else {
        pin = top ? LIMIT_TOP_M2 : LIMIT_BOT_M2;
    }
    
    return (gpio_get_level(pin) == 0);  // Active low
}

void safety_check(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_id_t motor_id = (motor_id_t)i;
        
        // Check current
        float current_ma = read_current(motor_id);
        
        if (motor_state_mutex != NULL) {
            xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
        }
        
        motor_states[motor_id].current_ma = current_ma;
        
        // Check over-current
        if (ABS(current_ma) > CURRENT_LIMIT_MA) {
            if (!motor_states[motor_id].over_current) {
                motor_states[motor_id].over_current = true;
                ESP_LOGW(TAG, "Motor %d over-current: %.0f mA", motor_id, current_ma);
                set_motor_speed(motor_id, 0);
            }
        } else {
            motor_states[motor_id].over_current = false;
        }
        
        // Check stall condition (no hall pulses for STALL_TIMEOUT_MS while moving)
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (motor_states[motor_id].current_speed != 0 &&
            (now - last_hall_time_ms[motor_id]) > STALL_TIMEOUT_MS) {
            ESP_LOGW(TAG, "Motor %d stall detected", motor_id);
            set_motor_speed(motor_id, 0);
        }
        
        if (motor_state_mutex != NULL) {
            xSemaphoreGive(motor_state_mutex);
        }
        
        // Check motor end red-yellow wire sensors
        bool red_alarm = false, yellow_alarm = false;
        float temp = read_motor_end_sensors(motor_id, &red_alarm, &yellow_alarm);
        
        motor_states[motor_id].red_wire_alarm = red_alarm;
        motor_states[motor_id].yellow_wire_alarm = yellow_alarm;
        motor_states[motor_id].motor_end_temp = temp;
        
        // Red wire alarm = primary thermal/overtemp shutdown
        if (red_alarm && !motor_states[motor_id].thermal_shutdown) {
            motor_states[motor_id].thermal_shutdown = true;
            ESP_LOGW(TAG, "Motor %d RED WIRE ALARM - Thermal shutdown: %.1f°C", motor_id, temp);
            set_motor_speed(motor_id, 0);
        }
        
        // Yellow wire alarm = secondary condition (may be different threshold)
        if (yellow_alarm) {
            ESP_LOGW(TAG, "Motor %d YELLOW WIRE ALARM detected", motor_id);
            // Depending on your sensor, this might also need emergency stop
            // set_motor_speed(motor_id, 0);
        }
    }
}

// ============================================================================
// HEIGHT CALIBRATION
// ============================================================================

void update_height(void) {
    int32_t avg_pulses = (hall_counters[MOTOR_ID_1] + hall_counters[MOTOR_ID_2]) / 2;
    float new_height = avg_pulses / PULSES_PER_MM;
    
    if (desk_state_mutex != NULL) {
        xSemaphoreTake(desk_state_mutex, portMAX_DELAY);
    }
    
    desk_state.height_mm = new_height;
    
    // Enforce limits
    if (desk_state.height_mm > MAX_HEIGHT_MM) {
        desk_state.height_mm = MAX_HEIGHT_MM;
        stop_motors();
        ESP_LOGW(TAG, "Maximum height reached");
    }
    if (desk_state.height_mm < MIN_HEIGHT_MM) {
        desk_state.height_mm = MIN_HEIGHT_MM;
        stop_motors();
        ESP_LOGW(TAG, "Minimum height reached");
    }
    
    // Update legacy variables
    heightMM = desk_state.height_mm;
    lastHallTime = (last_hall_time_ms[MOTOR_ID_1] > last_hall_time_ms[MOTOR_ID_2]) ?
                   last_hall_time_ms[MOTOR_ID_1] : last_hall_time_ms[MOTOR_ID_2];
    
    if (desk_state_mutex != NULL) {
        xSemaphoreGive(desk_state_mutex);
    }
}

float get_current_height(void) {
    update_height();
    return desk_state.height_mm;
}

void calibrate_height(float known_height_mm) {
    // Reset hall counters to match known height
    hall_counters[MOTOR_ID_1] = (int32_t)(known_height_mm * PULSES_PER_MM);
    hall_counters[MOTOR_ID_2] = (int32_t)(known_height_mm * PULSES_PER_MM);
    
    update_height();
    desk_state.calibrated = true;
    
    ESP_LOGI(TAG, "Calibrated at %.1f mm", known_height_mm);
}

// ============================================================================
// SYNCHRONIZATION
// ============================================================================

void sync_motors(int16_t base_speed) {
    // Get current positions
    int32_t pos1 = hall_counters[MOTOR_ID_1];
    int32_t pos2 = hall_counters[MOTOR_ID_2];
    
    // Calculate error
    int32_t error = pos1 - pos2;
    
    // P-controller adjustment
    int16_t adjustment = (int16_t)(error * SYNC_KP);
    
    // Limit adjustment
    adjustment = CLAMP(adjustment, -50, 50);
    
    // Apply to Motor 2
    int16_t new_speed = base_speed + adjustment;
    new_speed = CLAMP(new_speed, -PWM_MAX, PWM_MAX);
    
    set_motor_speed(MOTOR_ID_2, new_speed);
    
    ESP_LOGD(TAG, "Sync: pos1=%ld, pos2=%ld, error=%ld, adj=%d",
             pos1, pos2, error, adjustment);
}

// ============================================================================
// STATE ACCESS
// ============================================================================

void get_motor_state(motor_id_t motor_id, motor_state_t* state) {
    if (motor_id >= MOTOR_COUNT || state == NULL) return;
    
    if (motor_state_mutex != NULL) {
        xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
    }
    memcpy(state, &motor_states[motor_id], sizeof(motor_state_t));
    if (motor_state_mutex != NULL) {
        xSemaphoreGive(motor_state_mutex);
    }
}

void get_desk_state(desk_state_t* state) {
    if (state == NULL) return;
    
    update_height();
    
    if (desk_state_mutex != NULL) {
        xSemaphoreTake(desk_state_mutex, portMAX_DELAY);
    }
    memcpy(state, &desk_state, sizeof(desk_state_t));
    if (desk_state_mutex != NULL) {
        xSemaphoreGive(desk_state_mutex);
    }
}

// ============================================================================
// FREERTOS TASKS
// ============================================================================

void task_motor_control(void *param) {
    ESP_LOGI(TAG, "Motor control task started");
    
    const TickType_t xTicksToWait = pdMS_TO_TICKS(50);  // 50ms update rate
    
    while (1) {
        motor_command_t cmd;
        
        // Check for new commands
        if (xQueueReceive(motor_command_queue, &cmd, 0) == pdPASS) {
            if (motor_state_mutex != NULL) {
                xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
            }
            motor_states[cmd.motor_id].target_speed = cmd.speed;
            if (motor_state_mutex != NULL) {
                xSemaphoreGive(motor_state_mutex);
            }
            ESP_LOGI(TAG, "Received command: motor=%d, speed=%d", cmd.motor_id, cmd.speed);
        }
        
        // Smooth acceleration (max 10 units per update)
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motor_id_t motor_id = (motor_id_t)i;
            
            if (motor_state_mutex != NULL) {
                xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
            }
            int16_t current = motor_states[motor_id].current_speed;
            int16_t target = motor_states[motor_id].target_speed;
            if (motor_state_mutex != NULL) {
                xSemaphoreGive(motor_state_mutex);
            }
            
            int16_t new_speed;
            if (current < target) {
                new_speed = current + 10;
                if (new_speed > target) new_speed = target;
            } else if (current > target) {
                new_speed = current - 10;
                if (new_speed < target) new_speed = target;
            } else {
                continue;  // Already at target
            }
            
            set_motor_speed(motor_id, new_speed);
        }
        
        // Synchronize motors if moving
        if (motion_state != MOTION_STOPPED) {
            sync_motors(MOTOR_SPEED);
        }
        
        vTaskDelay(xTicksToWait);
    }
}

void task_current_monitor(void *param) {
    ESP_LOGI(TAG, "Current monitor task started");
    
    const TickType_t xTicksToWait = pdMS_TO_TICKS(100);  // 100ms check rate
    
    while (1) {
        safety_check();
        vTaskDelay(xTicksToWait);
    }
}

void task_hall_monitor(void *param) {
    ESP_LOGI(TAG, "Hall monitor task started");
    
    const TickType_t xTicksToWait = pdMS_TO_TICKS(1000);  // 1 second
    int32_t prev_counts[MOTOR_COUNT] = {0, 0};
    
    while (1) {
        vTaskDelay(xTicksToWait);
        
        for (int i = 0; i < MOTOR_COUNT; i++) {
            int32_t delta = hall_counters[i] - prev_counts[i];
            float rpm = (delta / 6.0f) * 60.0f;  // Assuming 6 Hall events per revolution
            
            if (motor_state_mutex != NULL) {
                xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
            }
            motor_states[i].hall_count = hall_counters[i];
            if (motor_state_mutex != NULL) {
                xSemaphoreGive(motor_state_mutex);
            }
            
            ESP_LOGD(TAG, "Motor %d: %ld edges/sec, ~%.1f RPM", i, delta, rpm);
        }
        
        prev_counts[0] = hall_counters[0];
        prev_counts[1] = hall_counters[1];
        
        // Update height
        update_height();
    }
}

void task_command_handler(void *param) {
    ESP_LOGI(TAG, "Command handler task started");
    
    // This task would normally handle button presses, web commands, etc.
    // For now, just idle
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_telemetry(void *param) {
    ESP_LOGI(TAG, "Telemetry task started");
    
    const TickType_t xTicksToWait = pdMS_TO_TICKS(1000);  // 1 second
    
    while (1) {
        vTaskDelay(xTicksToWait);
        
        motor_state_t states[MOTOR_COUNT];
        for (int i = 0; i < MOTOR_COUNT; i++) {
            get_motor_state((motor_id_t)i, &states[i]);
        }
        
        float height = get_current_height();
        
        ESP_LOGI(TAG, "=== MOTOR STATUS ===");
        ESP_LOGI(TAG, "Height: %.1f mm", height);
        for (int i = 0; i < MOTOR_COUNT; i++) {
            ESP_LOGI(TAG, "Motor %d | Speed: %d | Current: %.0f mA | OC: %s | TH: %s | LM: %s",
                     i,
                     states[i].current_speed,
                     states[i].current_ma,
                     states[i].over_current ? "YES" : "NO",
                     states[i].thermal_shutdown ? "YES" : "NO",
                     states[i].limit_triggered ? "YES" : "NO");
        }
    }
}
