#include "test_mocks.h"
#include <stdarg.h>
#include <stdio.h>

mock_state_t mock_state;
uint32_t last_hall_time_ms[2] = {0, 0};

void mock_init(void) {
    memset(&mock_state, 0, sizeof(mock_state));
    memset(last_hall_time_ms, 0, sizeof(last_hall_time_ms));
    // Set default GPIO levels to HIGH (1)
    for (int i = 0; i < 40; i++) {
        mock_state.gpio_levels[i] = 1;
    }
}

void mock_reset(void) {
    mock_init();
}

// FreeRTOS Mocks
QueueHandle_t mock_queue_create(int length, int size) {
    (void)length;
    (void)size;
    return (QueueHandle_t)0x1234;  // Dummy handle
}

SemaphoreHandle_t mock_semaphore_create_mutex(void) {
    return (SemaphoreHandle_t)0x5678;  // Dummy handle
}

int mock_semaphore_take(SemaphoreHandle_t mutex) {
    (void)mutex;
    mock_state.mutex_taken++;
    return pdTRUE;
}

int mock_semaphore_give(SemaphoreHandle_t mutex) {
    (void)mutex;
    mock_state.mutex_given++;
    return pdTRUE;
}

TickType_t mock_get_tick_count(void) {
    return mock_state.tick_count;
}

void mock_task_delay(TickType_t ticks) {
    mock_state.tick_count += ticks;
}

// GPIO Mocks
int mock_gpio_get_level(gpio_num_t pin) {
    if (pin >= 0 && pin < 40) {
        return mock_state.gpio_levels[pin];
    }
    return 0;
}

void mock_gpio_set_level(gpio_num_t pin, int level) {
    if (pin >= 0 && pin < 40) {
        mock_state.gpio_levels[pin] = level;
    }
}

void mock_gpio_configure(gpio_num_t pin, int mode) {
    (void)pin;
    (void)mode;
    // No-op in mock
}

// ADC Mocks
int mock_adc_get_raw(adc1_channel_t channel) {
    if (channel >= 0 && channel < 10) {
        return mock_state.adc_values[channel];
    }
    return 0;
}

void mock_adc_set_raw(adc1_channel_t channel, int value) {
    if (channel >= 0 && channel < 10) {
        mock_state.adc_values[channel] = value;
    }
}

// LEDC Mocks
void mock_ledc_set_duty(int channel, int duty) {
    if (channel >= 0 && channel < 10) {
        mock_state.ledc_duties[channel] = duty;
    }
}

int mock_ledc_get_duty(int channel) {
    if (channel >= 0 && channel < 10) {
        return mock_state.ledc_duties[channel];
    }
    return 0;
}

void mock_ledc_update_duty(int channel) {
    (void)channel;
    // No-op in mock
}

// Logging Mocks
void mock_log_info(const char* tag, const char* format, ...) {
    (void)tag;
    va_list args;
    va_start(args, format);
    printf("[INFO] %s: ", tag);
    vprintf(format, args);
    printf("\n");
    va_end(args);
    mock_state.log_count++;
}

void mock_log_warn(const char* tag, const char* format, ...) {
    (void)tag;
    va_list args;
    va_start(args, format);
    printf("[WARN] %s: ", tag);
    vprintf(format, args);
    printf("\n");
    va_end(args);
    mock_state.log_count++;
}

void mock_log_error(const char* tag, const char* format, ...) {
    (void)tag;
    va_list args;
    va_start(args, format);
    printf("[ERROR] %s: ", tag);
    vprintf(format, args);
    printf("\n");
    va_end(args);
    mock_state.log_count++;
}

void mock_log_debug(const char* tag, const char* format, ...) {
    (void)tag;
    va_list args;
    va_start(args, format);
    printf("[DEBUG] %s: ", tag);
    vprintf(format, args);
    printf("\n");
    va_end(args);
    mock_state.log_count++;
}
