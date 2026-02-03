#include "motor/display.h"
#include "motor/pin.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "display";

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

nvs_handle_t nvs_handle = 0;

char wifi_ssid[32] = "DeskController";
char wifi_password[64] = "desk123456";
bool wifi_connected = false;

static httpd_handle_t http_server = NULL;

// Button debounce
static uint32_t last_button_time[4] = {0, 0, 0, 0};
static bool button_pressed[4] = {false, false, false, false};

// Display buffer (simple character buffer)
static char display_buffer[4][21];  // 4 lines, 20 chars each

// ============================================================================
// INITIALIZATION
// ============================================================================

void storage_init(void) {
    ESP_LOGI(TAG, "Initializing storage");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(ret));
    }
    
    // Load saved heights
    float h1 = storage_load_height(NVS_KEY_M1_POS);
    float h2 = storage_load_height(NVS_KEY_M2_POS);
    
    if (h1 >= 0) desk_state.mem1_height = h1;
    if (h2 >= 0) desk_state.mem2_height = h2;
    
    ESP_LOGI(TAG, "Storage initialized");
}

void i2c_init(void) {
    ESP_LOGI(TAG, "Initializing I2C");
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    
    ESP_LOGI(TAG, "I2C initialized");
}

void display_init(void) {
    ESP_LOGI(TAG, "Initializing display");
    
    // Clear display buffer
    for (int i = 0; i < 4; i++) {
        memset(display_buffer[i], ' ', 20);
        display_buffer[i][20] = '\0';
    }
    
    // Display welcome message
    snprintf(display_buffer[0], 20, "Desk Controller");
    snprintf(display_buffer[1], 20, "Initializing...");
    
    ESP_LOGI(TAG, "Display initialized");
}

void buttons_init(void) {
    ESP_LOGI(TAG, "Initializing buttons");
    
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BTN_UP) | (1ULL << BTN_DOWN) |
                       (1ULL << BTN_M1) | (1ULL << BTN_M2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn_conf));
    
    ESP_LOGI(TAG, "Buttons initialized");
}

// ============================================================================
// STORAGE
// ============================================================================

float storage_load_height(const char* key) {
    int32_t height_mm_x100 = -1;
    esp_err_t ret = nvs_get_i32(nvs_handle, key, &height_mm_x100);
    
    if (ret == ESP_OK) {
        return height_mm_x100 / 100.0f;
    }
    return -1.0f;
}

esp_err_t storage_save_height(const char* key, float height) {
    int32_t height_mm_x100 = (int32_t)(height * 100);
    return nvs_set_i32(nvs_handle, key, height_mm_x100);
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void buttons_poll(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    gpio_num_t buttons[4] = {BTN_UP, BTN_DOWN, BTN_M1, BTN_M2};
    
    for (int i = 0; i < 4; i++) {
        bool pressed = (gpio_get_level(buttons[i]) == 0);  // Active low
        
        if (pressed) {
            if ((now - last_button_time[i]) > DEBOUNCE_MS && !button_pressed[i]) {
                button_pressed[i] = true;
                last_button_time[i] = now;
                
                // Handle button press
                switch (i) {
                    case 0:  // UP
                        ESP_LOGI(TAG, "Button UP pressed");
                        move_up();
                        break;
                    case 1:  // DOWN
                        ESP_LOGI(TAG, "Button DOWN pressed");
                        move_down();
                        break;
                    case 2:  // M1
                        ESP_LOGI(TAG, "Button M1 pressed - saving height");
                        desk_state.mem1_height = get_current_height();
                        storage_save_height(NVS_KEY_M1_POS, desk_state.mem1_height);
                        break;
                    case 3:  // M2
                        ESP_LOGI(TAG, "Button M2 pressed - saving height");
                        desk_state.mem2_height = get_current_height();
                        storage_save_height(NVS_KEY_M2_POS, desk_state.mem2_height);
                        break;
                }
            }
        } else {
            button_pressed[i] = false;
        }
    }
}

bool button_was_pressed(uint8_t button_id) {
    if (button_id >= 4) return false;
    bool pressed = button_pressed[button_id];
    button_pressed[button_id] = false;  // Clear after reading
    return pressed;
}

// ============================================================================
// DISPLAY
// ============================================================================

void display_update(void) {
    float height = get_current_height();
    
    // Update display buffer
    snprintf(display_buffer[0], 20, "Height: %6.1f mm", height);
    snprintf(display_buffer[1], 20, "M1:%4d M2:%4d", 
             motor_states[MOTOR_ID_1].current_speed,
             motor_states[MOTOR_ID_2].current_speed);
    
    // Show current states
    char status[20] = "";
    if (motion_state == MOTION_MOVING_UP) {
        strncpy(status, "MOVING UP  ", 20);
    } else if (motion_state == MOTION_MOVING_DOWN) {
        strncpy(status, "MOVING DOWN", 20);
    } else {
        strncpy(status, "STOPPED    ", 20);
    }
    
    snprintf(display_buffer[2], 20, "%s", status);
    
    // Show memory positions
    if (desk_state.mem1_height >= 0 && desk_state.mem2_height >= 0) {
        snprintf(display_buffer[3], 20, "M1:%3.0f M2:%3.0f", 
                 desk_state.mem1_height, desk_state.mem2_height);
    } else {
        snprintf(display_buffer[3], 20, "No saved positions");
    }
}

void display_error(const char* msg) {
    ESP_LOGE(TAG, "Display error: %s", msg);
    snprintf(display_buffer[1], 20, "ERROR: %s", msg);
}

void display_height(float height_mm) {
    snprintf(display_buffer[0], 20, "Height: %6.1f mm", height_mm);
}

void display_motor_status(const motor_state_t* m1, const motor_state_t* m2) {
    ESP_LOGI(TAG, "M1: spd=%d cur=%.0fmA | M2: spd=%d cur=%.0fmA",
             m1->current_speed, m1->current_ma,
             m2->current_speed, m2->current_ma);
}

// ============================================================================
// WEB SERVER HANDLERS
// ============================================================================

static esp_err_t web_root_handler(httpd_req_t *req) {
    const char* html = 
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<title>Desk Controller</title>"
        "<style>"
        "body { font-family: Arial; margin: 20px; background: #f0f0f0; }"
        ".container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }"
        "h1 { color: #333; }"
        ".height { font-size: 48px; text-align: center; margin: 20px 0; color: #2196F3; }"
        ".controls { display: flex; justify-content: center; gap: 20px; margin: 20px 0; }"
        "button { padding: 20px 40px; font-size: 24px; border: none; border-radius: 10px; cursor: pointer; }"
        ".up { background: #4CAF50; color: white; }"
        ".down { background: #f44336; color: white; }"
        ".stop { background: #ff9800; color: white; }"
        ".status { background: #e3f2fd; padding: 15px; border-radius: 5px; margin: 10px 0; }"
        ".memory { display: flex; gap: 10px; justify-content: center; margin: 20px 0; }"
        ".mem-btn { padding: 10px 30px; font-size: 16px; background: #9c27b0; color: white; }"
        "</style>"
        "</head>"
        "<body>"
        "<div class='container'>"
        "<h1>Desk Controller</h1>"
        "<div class='height' id='height'>-- mm</div>"
        "<div class='controls'>"
        "<button class='up' onclick='move(\"up\")'>UP</button>"
        "<button class='stop' onclick='move(\"stop\")'>STOP</button>"
        "<button class='down' onclick='move(\"down\")'>DOWN</button>"
        "</div>"
        "<div class='memory'>"
        "<button class='mem-btn' onclick='goTo(1)'>Memory 1</button>"
        "<button class='mem-btn' onclick='goTo(2)'>Memory 2</button>"
        "</div>"
        "<div class='status' id='status'>Loading...</div>"
        "</div>"
        "<script>"
        "function move(dir) {"
        "  fetch('/api/move?dir=' + dir, {method: 'POST'});"
        "}"
        "function goTo(mem) {"
        "  fetch('/api/goto?mem=' + mem, {method: 'POST'});"
        "}"
        "function updateStatus() {"
        "  fetch('/api/status').then(r => r.json()).then(data => {"
        "    document.getElementById('height').textContent = data.height.toFixed(1) + ' mm';"
        "    let s = 'Motor 1: ' + data.motor0.speed + ' | ' + data.motor0.current.toFixed(0) + 'mA<br>';"
        "    s += 'Motor 2: ' + data.motor1.speed + ' | ' + data.motor1.current.toFixed(0) + 'mA';"
        "    document.getElementById('status').innerHTML = s;"
        "  });"
        "}"
        "setInterval(updateStatus, 500);"
        "updateStatus();"
        "</script>"
        "</body>"
        "</html>";
    
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static esp_err_t web_api_move_handler(httpd_req_t *req) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char dir[16];
        if (httpd_query_key_value(query, "dir", dir, sizeof(dir)) == ESP_OK) {
            if (strcmp(dir, "up") == 0) {
                move_up();
            } else if (strcmp(dir, "down") == 0) {
                move_down();
            } else if (strcmp(dir, "stop") == 0) {
                stop_motors();
            }
        }
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t web_api_goto_handler(httpd_req_t *req) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char mem[8];
        if (httpd_query_key_value(query, "mem", mem, sizeof(mem)) == ESP_OK) {
            int mem_num = atoi(mem);
            float target = -1;
            
            if (mem_num == 1 && desk_state.mem1_height >= 0) {
                target = desk_state.mem1_height;
            } else if (mem_num == 2 && desk_state.mem2_height >= 0) {
                target = desk_state.mem2_height;
            }
            
            if (target >= 0) {
                // Queue a height move command (would need to be implemented)
                ESP_LOGI(TAG, "Moving to memory %d: %.1f mm", mem_num, target);
            }
        }
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t web_api_status_handler(httpd_req_t *req) {
    motor_state_t m1, m2;
    get_motor_state(MOTOR_ID_1, &m1);
    get_motor_state(MOTOR_ID_2, &m2);
    
    float height = get_current_height();
    
    char json[256];
    snprintf(json, sizeof(json),
        "{"
        "\"height\":%.1f,"
        "\"motor0\":{\"speed\":%d,\"current\":%.1f,\"oc\":%s}," \
        "\"motor1\":{\"speed\":%d,\"current\":%.1f,\"oc\":%s}"
        "}",
        height,
        m1.current_speed, m1.current_ma, m1.over_current ? "true" : "false",
        m2.current_speed, m2.current_ma, m2.over_current ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

void wifi_init(void) {
    ESP_LOGI(TAG, "Initializing WiFi");
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "DeskController",
            .password = "desk123456",
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialized, connecting...");
}

void webserver_init(void) {
    ESP_LOGI(TAG, "Initializing web server");
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    
    if (httpd_start(&http_server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = web_root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server, &root_uri);
        
        httpd_uri_t move_uri = {
            .uri = "/api/move",
            .method = HTTP_POST,
            .handler = web_api_move_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server, &move_uri);
        
        httpd_uri_t goto_uri = {
            .uri = "/api/goto",
            .method = HTTP_POST,
            .handler = web_api_goto_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server, &goto_uri);
        
        httpd_uri_t status_uri = {
            .uri = "/api/status",
            .method = HTTP_GET,
            .handler = web_api_status_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server, &status_uri);
        
        ESP_LOGI(TAG, "Web server started on port %d", WEB_SERVER_PORT);
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
    }
}

// ============================================================================
// FREERTOS TASKS
// ============================================================================

void task_display(void *param) {
    ESP_LOGI(TAG, "Display task started");
    
    while (1) {
        display_update();
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_UPDATE_MS));
    }
}

void task_buttons(void *param) {
    ESP_LOGI(TAG, "Button task started");
    
    while (1) {
        buttons_poll();
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

void task_webserver(void *param) {
    ESP_LOGI(TAG, "Web server task started");
    
    wifi_init();
    webserver_init();
    
    // Keep task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// JSON BUILDERS (for external use)
// ============================================================================

int build_status_json(char* buf, size_t bufsize) {
    motor_state_t m1, m2;
    get_motor_state(MOTOR_ID_1, &m1);
    get_motor_state(MOTOR_ID_2, &m2);
    
    float height = get_current_height();
    
    return snprintf(buf, bufsize,
        "{"
        "\"height\":%.1f,"
        "\"motion\":%d,"
        "\"calibrated\":%s,"
        "\"mem1\":%.1f,"
        "\"mem2\":%.1f"
        "}",
        height,
        motion_state,
        desk_state.calibrated ? "true" : "false",
        desk_state.mem1_height,
        desk_state.mem2_height);
}

int build_motor_json(char* buf, size_t bufsize, const motor_state_t* states) {
    return snprintf(buf, bufsize,
        "["
        "{\"speed\":%d,\"current\":%.1f,\"hall\":%d,\"oc\":%s,\"th\":%s,\"lm\":%s},"
        "{\"speed\":%d,\"current\":%.1f,\"hall\":%d,\"oc\":%s,\"th\":%s,\"lm\":%s}"
        "]",
        states[0].current_speed, states[0].current_ma, states[0].hall_count,
        states[0].over_current ? "true" : "false",
        states[0].thermal_shutdown ? "true" : "false",
        states[0].limit_triggered ? "true" : "false",
        states[1].current_speed, states[1].current_ma, states[1].hall_count,
        states[1].over_current ? "true" : "false",
        states[1].thermal_shutdown ? "true" : "false",
        states[1].limit_triggered ? "true" : "false");
}
