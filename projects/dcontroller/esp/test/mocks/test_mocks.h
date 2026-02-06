#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ============================================================================
// Mock FreeRTOS Types
// ============================================================================
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

// ============================================================================
// Mock FreeRTOS Functions
// ============================================================================
#define xQueueCreate(length, size) mock_queue_create(length, size)
#define xSemaphoreCreateMutex() mock_semaphore_create_mutex()
#define xSemaphoreTake(mutex, timeout) mock_semaphore_take(mutex)
#define xSemaphoreGive(mutex) mock_semaphore_give(mutex)
#define xTaskGetTickCount() mock_get_tick_count()
#define vTaskDelay(ticks) mock_task_delay(ticks)
#define pdMS_TO_TICKS(ms) (ms)

// ============================================================================
// Mock Atomic Operations
// ============================================================================
#define _Atomic
#define atomic_store(obj, desired) do { *(obj) = (desired); } while(0)
#define atomic_load(obj) (*(obj))
#define atomic_fetch_add(obj, value) ({ typeof(*(obj)) old = *(obj); *(obj) += (value); old; })

// ============================================================================
// Mock GPIO Types & Constants
// ============================================================================
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

// GPIO modes
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_PULLUP_ENABLE 1
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_ENABLE 1
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0
#define GPIO_INTR_POSEDGE 1
#define GPIO_INTR_NEGEDGE 2

typedef struct {
    uint64_t pin_bit_mask;
    int mode;
    int pull_up_en;
    int pull_down_en;
    int intr_type;
} gpio_config_t;

// Mock GPIO Functions
#define gpio_get_level(pin) mock_gpio_get_level(pin)
#define gpio_set_level(pin, level) mock_gpio_set_level(pin, level)
#define gpio_config(conf) ESP_OK
#define gpio_install_isr_service(flags) ESP_OK
#define gpio_isr_handler_add(pin, handler, arg) ESP_OK

// ============================================================================
// Mock ADC Types & Constants
// ============================================================================
typedef int adc1_channel_t;
typedef int adc_channel_t;
typedef int adc_unit_t;
typedef int adc_atten_t;
typedef int adc_bitwidth_t;
typedef int adc_ulp_mode_t;
typedef int adc_rtc_clk_src_t;

#define ADC_UNIT_1 1
#define ADC_CHANNEL_4 4
#define ADC_CHANNEL_5 5
#define ADC_ATTEN_DB_12 0
#define ADC_BITWIDTH_12 12
#define ADC_ULP_MODE_DISABLE 0
#define ADC_RTC_CLK_SRC_DEFAULT 0

// Current sensor channels
#define CUR_M1 ADC_CHANNEL_4
#define CUR_M2 ADC_CHANNEL_5

// ADC handles
typedef void* adc_oneshot_unit_handle_t;
typedef void* adc_cali_handle_t;

typedef struct {
    adc_unit_t unit_id;
    adc_rtc_clk_src_t clk_src;
    adc_ulp_mode_t ulp_mode;
} adc_oneshot_unit_init_cfg_t;

typedef struct {
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
} adc_oneshot_chan_cfg_t;

typedef struct {
    adc_unit_t unit_id;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
    int default_vref;
} adc_cali_line_fitting_config_t;

// Mock ADC Functions
#define adc1_get_raw(channel) mock_adc_get_raw(channel)
#define adc_oneshot_new_unit(cfg, handle) ESP_OK
#define adc_oneshot_config_channel(handle, channel, cfg) ESP_OK
#define adc_oneshot_read(handle, channel, raw) ({ \
    *(raw) = mock_adc_get_raw(channel); \
    ESP_OK; \
})
#define adc_cali_create_scheme_line_fitting(cfg, handle) ESP_OK
#define adc_cali_raw_to_voltage(cali, raw, voltage) ({ \
    *(voltage) = (raw * 3300) / 4095; \
    ESP_OK; \
})

// ============================================================================
// Mock LEDC Types & Constants
// ============================================================================
typedef int ledc_mode_t;
typedef int ledc_timer_t;
typedef int ledc_channel_t;
typedef int ledc_timer_bit_t;
typedef int ledc_intr_type_t;
typedef int ledc_clk_cfg_t;
typedef int ledc_sleep_mode_t;

#define LEDC_LOW_SPEED_MODE 0
#define LEDC_TIMER_0 0
#define LEDC_TIMER_8_BIT 8
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_INTR_DISABLE 0
#define LEDC_AUTO_CLK 0
#define LEDC_SLEEP_MODE_NO_ALIVE_NO_PD 0

typedef struct {
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t timer_num;
    uint32_t freq_hz;
    ledc_clk_cfg_t clk_cfg;
    bool deconfigure;
} ledc_timer_config_t;

typedef struct {
    int gpio_num;
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    ledc_intr_type_t intr_type;
    ledc_timer_t timer_sel;
    uint32_t duty;
    int hpoint;
    ledc_sleep_mode_t sleep_mode;
    struct {} flags;
} ledc_channel_config_t;

// Mock LEDC Functions
#define ledc_timer_config(cfg) ESP_OK
#define ledc_channel_config(cfg) ESP_OK
#define ledc_set_duty(mode, channel, duty) mock_ledc_set_duty(channel, duty)
#define ledc_update_duty(mode, channel) mock_ledc_update_duty(channel)

// ============================================================================
// Mock ESP Log
// ============================================================================
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

// ============================================================================
// Mock NVS
// ============================================================================
typedef int nvs_handle_t;
#define NVS_READWRITE 0

#define nvs_flash_init() ESP_OK
#define nvs_flash_erase() ESP_OK
#define nvs_open(name, mode, handle) ({ *(handle) = 1; ESP_OK; })
#define nvs_get_i32(handle, key, value) ESP_ERR_NVS_NOT_FOUND
#define nvs_set_i32(handle, key, value) ESP_OK
#define nvs_close(handle)
#define esp_err_to_name(err) "ESP_OK"

// ============================================================================
// Mock I2C
// ============================================================================
typedef int i2c_port_t;
typedef int i2c_mode_t;

#define I2C_NUM_0 0
#define I2C_MODE_MASTER 1
#define I2C_FREQ 400000

typedef struct {
    i2c_mode_t mode;
    int sda_io_num;
    int scl_io_num;
    int sda_pullup_en;
    int scl_pullup_en;
    struct { int clk_speed; } master;
    int clk_flags;
} i2c_config_t;

#define i2c_param_config(port, conf) ESP_OK
#define i2c_driver_install(port, mode, rx, tx, flags) ESP_OK

// ============================================================================
// Mock WiFi
// ============================================================================
typedef int wifi_mode_t;
typedef int wifi_interface_t;
typedef int wifi_scan_method_t;

#define WIFI_MODE_STA 0
#define WIFI_IF_STA 0
#define WIFI_FAST_SCAN 0

typedef struct {
    struct {
        char ssid[32];
        char password[64];
        wifi_scan_method_t scan_method;
        char sae_h2e_identifier[1];
    } sta;
} wifi_config_t;

typedef struct {} wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() (wifi_init_config_t){}

#define esp_netif_init() ESP_OK
#define esp_event_loop_create_default() ESP_OK
#define esp_netif_create_default_wifi_sta() NULL
#define esp_wifi_init(cfg) ESP_OK
#define esp_wifi_set_mode(mode) ESP_OK
#define esp_wifi_set_config(iface, conf) ESP_OK
#define esp_wifi_start() ESP_OK

// ============================================================================
// Mock HTTP Server
// ============================================================================
typedef void* httpd_handle_t;
typedef struct {} httpd_req_t;
typedef int httpd_method_t;

#define HTTP_GET 1
#define HTTP_POST 2
#define HTTPD_DEFAULT_CONFIG() (httpd_config_t){ .server_port = 80 }

typedef struct {
    int server_port;
} httpd_config_t;

typedef struct {
    const char* uri;
    httpd_method_t method;
    void* handler;
    void* user_ctx;
} httpd_uri_t;

#define httpd_start(handle, config) ESP_OK
#define httpd_register_uri_handler(server, uri) ESP_OK
#define httpd_resp_send(req, data, len) ESP_OK
#define httpd_resp_set_type(req, type) ESP_OK
#define httpd_req_get_url_query_str(req, buf, len) 1
#define httpd_query_key_value(query, key, val, len) 0

// ============================================================================
// Mock Timer
// ============================================================================
#define esp_timer_get_time() (mock_state.tick_count * 1000ULL)
#define esp_rom_delay_us(us) do { mock_state.tick_count += ((us) / 1000) + 1; } while(0)

// ============================================================================
// Mock Implementation Functions
// ============================================================================
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

// ============================================================================
// Mock State Tracking
// ============================================================================
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

// ============================================================================
// External Variables for Test Access
// ============================================================================
extern uint32_t last_hall_time_ms[2];
