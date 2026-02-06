#include "motor/pin.h"
#include "motor/motor.h"
#include "motor/display.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>

static const char *TAG = "main";

// ============================================================================
// MAIN APPLICATION
// ============================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32 Dual Motor d Controller");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize NVS first (needed for storage and WiFi)
    storage_init();
    
    // Initialize all hardware
    ESP_LOGI(TAG, "Initializing hardware...");
    motor_init();
    sensors_init();
    buttons_init();
    i2c_init();
    display_init();
    
    // Create FreeRTOS synchronization primitives
    motor_command_queue = xQueueCreate(10, sizeof(motor_command_t));
    if (motor_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create motor command queue");
        return;
    }
    
    motor_state_mutex = xSemaphoreCreateMutex();
    if (motor_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create motor state mutex");
        return;
    }
    
    d_state_mutex = xSemaphoreCreateMutex();
    if (d_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create d state mutex");
        return;
    }
    
    ESP_LOGI(TAG, "FreeRTOS primitives created");
    
    // Create all tasks
    ESP_LOGI(TAG, "Creating FreeRTOS tasks...");
    
    // Motor control task - high priority, Core 0
    BaseType_t ret = xTaskCreatePinnedToCore(
        task_motor_control,
        "MotorCtrl",
        TASK_MOTOR_CTRL_STACK,
        NULL,
        TASK_MOTOR_CTRL_PRIO,
        NULL,
        CORE_0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor control task");
    }
    
    // Current monitoring task - high priority, Core 0
    ret = xTaskCreatePinnedToCore(
        task_current_monitor,
        "CurrentMon",
        TASK_CURRENT_MON_STACK,
        NULL,
        TASK_CURRENT_MON_PRIO,
        NULL,
        CORE_0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create current monitor task");
    }
    
    // Hall sensor monitoring task - Core 1
    ret = xTaskCreatePinnedToCore(
        task_hall_monitor,
        "HallMon",
        TASK_HALL_MON_STACK,
        NULL,
        TASK_HALL_MON_PRIO,
        NULL,
        CORE_1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create hall monitor task");
    }
    
    // Command handler task - Core 1
    ret = xTaskCreatePinnedToCore(
        task_command_handler,
        "CmdHandler",
        TASK_CMD_HANDLER_STACK,
        NULL,
        TASK_CMD_HANDLER_PRIO,
        NULL,
        CORE_1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create command handler task");
    }
    
    // Telemetry task - Core 1
    ret = xTaskCreatePinnedToCore(
        task_telemetry,
        "Telemetry",
        TASK_TELEMETRY_STACK,
        NULL,
        TASK_TELEMETRY_PRIO,
        NULL,
        CORE_1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telemetry task");
    }
    
    // Display update task - Core 1
    ret = xTaskCreatePinnedToCore(
        task_display,
        "Display",
        TASK_DISPLAY_STACK,
        NULL,
        TASK_DISPLAY_PRIO,
        NULL,
        CORE_1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
    }
    
    // Button polling task - Core 0
    ret = xTaskCreatePinnedToCore(
        task_buttons,
        "Buttons",
        TASK_BUTTON_STACK,
        NULL,
        TASK_BUTTON_PRIO,
        NULL,
        CORE_0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
    }
    
    // Web server task - Core 1
    ret = xTaskCreatePinnedToCore(
        task_webserver,
        "WebServer",
        TASK_WEBSERVER_STACK,
        NULL,
        TASK_WEBSERVER_PRIO,
        NULL,
        CORE_1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create web server task");
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "All tasks created successfully!");
    ESP_LOGI(TAG, "System ready.");
    ESP_LOGI(TAG, "========================================");
    
    // Main loop - just delay forever (tasks do all the work)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // 10 second delay
        
        // Optional: Check system health here
        ESP_LOGD(TAG, "Main task heartbeat - System OK");
    }
}
