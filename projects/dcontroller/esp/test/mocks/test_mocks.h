#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Mock FreeRTOS Types
typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef uint32_t TickType_t;
typedef int BaseType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdFAIL 0
#define portMAX_DELAY 0xFFFFFFFF
#define portTICK_PERIOD_MS 1

// Mock FreeRTOS Functions
#define xQueueCreate(length, size) mock_queue_create(length, size)
#define xSemaphoreCreateMutex() mock_semaphore_create_mutex()
#define xSemaphoreTake(mutex, timeout) mock_semaphore_take(mutex)
#define xSemaphoreGive(mutex) mock_semaphore_give(mutex)
#define xTaskGetTickCount() mock_get_tick_count()
#define vTaskDelay(ticks) mock_task_delay(ticks)
#define pdMS_TO_TICKS(ms) (ms)

// Mock GPIO Types
typedef int gpio_num_t;
#define GPIO_NUM_0 0
#define GPIO_NUM_2 2
#define GPIO_NUM_4 4
#define GPIO_NUM_5 5
#define GPIO_NUM_14 14
#define GPIO_NUM_16 16
#define GPIO_NUM_17 17
#define GPIO_NUM_18 18
#define GPIO_NUM_19 19
#define GPIO_NUM_20 20
#define GPIO_NUM_21 21
#define GPIO_NUM_22 22
#define GPIO_NUM_23 23
#define GPIO_NUM_25 25
#define GPIO_NUM_26 26
#define GPIO_NUM_27 27
#define GPIO_NUM_32 32
#define GPIO_NUM_33 33
#define GPIO_NUM_34 34
#define GPIO_NUM_35 35

// Mock GPIO Functions
#define gpio_get_level(pin) mock_gpio_get_level(pin)
#define gpio_set_level(pin, level) mock_gpio_set_level(pin, level)

// Mock ADC Types
typedef int adc1_channel_t;
#define ADC1_CHANNEL_4 4
#define ADC1_CHANNEL_5 5
#define ADC_ATTEN_DB_11 0
#define ADC_WIDTH_BIT_12 12

// Mock ADC Functions
#define adc1_get_raw(channel) mock_adc_get_raw(channel)

// Mock LEDC
#define ledc_set_duty(mode, channel, duty) mock_ledc_set_duty(channel, duty)
#define ledc_update_duty(mode, channel) mock_ledc_update_duty(channel)

// Mock ESP Log
#define ESP_LOGI(tag, format, ...) mock_log_info(tag, format, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) mock_log_warn(tag, format, ##__VA_ARGS__)
#define ESP_LOGE(tag, format, ...) mock_log_error(tag, format, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) mock_log_debug(tag, format, ##__VA_ARGS__)

#define ESP_ERROR_CHECK(x) do { \
    esp_err_t err = (x); \
    if (err != ESP_OK) { \
        mock_log_error("ESP_ERROR_CHECK", "Error: %d", err); \
    } \
} while(0)

typedef int esp_err_t;
#define ESP_OK 0

// Mock implementation functions (defined in test_mocks.c)
void mock_init(void);
void mock_reset(void);

QueueHandle_t mock_queue_create(int length, int size);
SemaphoreHandle_t mock_semaphore_create_mutex(void);
int mock_semaphore_take(SemaphoreHandle_t mutex);
int mock_semaphore_give(SemaphoreHandle_t mutex);
TickType_t mock_get_tick_count(void);
void mock_task_delay(TickType_t ticks);

int mock_gpio_get_level(gpio_num_t pin);
void mock_gpio_set_level(gpio_num_t pin, int level);
void mock_gpio_configure(gpio_num_t pin, int mode);

int mock_adc_get_raw(adc1_channel_t channel);
void mock_adc_set_raw(adc1_channel_t channel, int value);

void mock_ledc_set_duty(int channel, int duty);
int mock_ledc_get_duty(int channel);
void mock_ledc_update_duty(int channel);

void mock_log_info(const char* tag, const char* format, ...);
void mock_log_warn(const char* tag, const char* format, ...);
void mock_log_error(const char* tag, const char* format, ...);
void mock_log_debug(const char* tag, const char* format, ...);

// Mock state tracking for tests
typedef struct {
    int gpio_levels[40];
    int adc_values[10];
    int ledc_duties[10];
    uint32_t tick_count;
    int mutex_taken;
    int mutex_given;
    int queue_items;
    int log_count;
} mock_state_t;

extern mock_state_t mock_state;
