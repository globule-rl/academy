#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <motor/motor.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_UPDATE_MS       100     // Display refresh interval
#define BUTTON_POLL_MS          50      // Button polling interval
#define WEB_SERVER_PORT         80

// ============================================================================
// EXTERNAL VARIABLES
// ============================================================================

// WiFi configuration
extern char wifi_ssid[32];
extern char wifi_password[64];
extern bool wifi_connected;

// ============================================================================
// FUNCTION DECLARATIONS - INITIALIZATION
// ============================================================================

/**
 * @brief Initialize NVS (non-volatile storage)
 */
void storage_init(void);

/**
 * @brief Initialize I2C for display
 */
void i2c_init(void);

/**
 * @brief Initialize OLED display
 */
void display_init(void);

/**
 * @brief Initialize WiFi in STA mode
 */
void wifi_init(void);

/**
 * @brief Initialize HTTP web server
 */
void webserver_init(void);

// ============================================================================
// FUNCTION DECLARATIONS - DISPLAY
// ============================================================================

/**
 * @brief Update display with current height and status
 */
void display_update(void);

/**
 * @brief Display error message
 * @param msg Error message string
 */
void display_error(const char* msg);

/**
 * @brief Display current height
 * @param height_mm Height in millimeters
 */
void display_height(float height_mm);

/**
 * @brief Display motor status
 * @param m1 Motor 1 state
 * @param m2 Motor 2 state
 */
void display_motor_status(const motor_state_t* m1, const motor_state_t* m2);

// ============================================================================
// FUNCTION DECLARATIONS - BUTTON HANDLING
// ============================================================================

/**
 * @brief Initialize button GPIOs
 */
void buttons_init(void);

/**
 * @brief Poll buttons and handle presses (call from task)
 */
void buttons_poll(void);

/**
 * @brief Get last button press state
 * @return true if button was pressed since last call
 */
bool button_was_pressed(uint8_t button_id);

/**
 * @brief Load saved height from NVS
 * @param key NVS key
 * @return Saved height or -1 if not found
 */
float storage_load_height(const char* key);

/**
 * @brief Save height to NVS
 * @param key NVS key
 * @param height Height value
 * @return ESP_OK on success
 */
esp_err_t storage_save_height(const char* key, float height);

// ============================================================================
// FUNCTION DECLARATIONS - WEB SERVER HANDLERS
// ============================================================================

/**
 * @brief Handle HTTP GET root request (serve control page)
 */
void web_handle_root(void);

/**
 * @brief Handle HTTP POST motor control request
 */
void web_handle_control(void);

/**
 * @brief Handle HTTP GET status request
 */
void web_handle_status(void);

/**
 * @brief Handle HTTP GET height request
 */
void web_handle_get_height(void);

/**
 * @brief Handle HTTP POST set height request
 */
void web_handle_set_height(void);

// ============================================================================
// FUNCTION DECLARATIONS - FREERTOS TASKS
// ============================================================================

/**
 * @brief Display update task
 */
void task_display(void *param);

/**
 * @brief Button polling task
 */
void task_buttons(void *param);

/**
 * @brief Web server task
 */
void task_webserver(void *param);

// ============================================================================
// JSON RESPONSE BUILDERS
// ============================================================================

/**
 * @brief Build JSON status response
 * @param buf Buffer to write JSON
 * @param bufsize Buffer size
 * @return Number of bytes written
 */
int build_status_json(char* buf, size_t bufsize);

/**
 * @brief Build JSON motor state response
 * @param buf Buffer to write JSON
 * @param bufsize Buffer size
 * @return Number of bytes written
 */
int build_motor_json(char* buf, size_t bufsize, const motor_state_t* states);

#ifdef __cplusplus
}
#endif
