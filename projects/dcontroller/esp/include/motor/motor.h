#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "motor/pin.h"

#ifdef __cplusplus
extern "C" {
#endif


// ============================================================================
// TYPE DEFINITIONS
// ============================================================================

typedef enum {
MOTION_STOPPED = 0,
MOTION_MOVING_UP,
MOTION_MOVING_DOWN
} motion_state_t;

typedef enum {
MOTOR_ID_1 = 0,
MOTOR_ID_2 = 1,
MOTOR_COUNT = 2
} motor_id_t;

typedef struct {
int16_t speed;          // -255 to 255
motor_id_t motor_id;
} motor_command_t;

typedef struct {
int16_t current_speed;      // Actual current speed
int16_t target_speed;       // Target speed
float current_ma;           // Current in mA
int32_t hall_count;         // Hall sensor pulse count
bool over_current;          // Over-current flag
bool thermal_shutdown;      // Thermal shutdown flag (from red wire)
bool limit_triggered;       // Limit switch triggered
bool red_wire_alarm;        // Red wire sensor triggered
bool yellow_wire_alarm;     // Yellow wire sensor triggered
float motor_end_temp;       // Calculated temperature from red-yellow wire pair
} motor_state_t;

typedef struct {
float height_mm;        // Current d height
float target_height_mm; // Target height for preset moves
float mem1_height;      // Memory position 1
float mem2_height;      // Memory position 2
bool calibrated;        // Height calibration status
} d_state_t;

// ============================================================================
// EXTERNAL VARIABLES
// ============================================================================

// System state
extern d_state_t d_state;

// FreeRTOS primitives
extern QueueHandle_t motor_command_queue;
extern SemaphoreHandle_t motor_state_mutex;
extern SemaphoreHandle_t d_state_mutex;

// ============================================================================
// FUNCTION DECLARATIONS - MOTOR CONTROL
// ============================================================================

/**
 * @brief Initialize motor hardware (GPIO, PWM)
 */
void motor_init(void);

/**
 * @brief Set motor speed and direction
 * @param motor_id Motor ID (0 or 1)
 * @param speed Speed -255 to 255 (negative = reverse)
 */
void set_motor_speed(motor_id_t motor_id, int16_t speed);

/**
 * @brief Stop both motors immediately
 */
void stop_motors(void);

/**
 * @brief Move d up
 */
void move_up(void);

/**
 * @brief Move d down
 */
void move_down(void);

/**
 * @brief Move to specific height (blocking with timeout)
 * @param target_height_mm Target height in millimeters
 * @param timeout_ms Timeout in milliseconds
 * @return true if reached target, false if timeout or error
 */
bool move_to_height(float target_height_mm, uint32_t timeout_ms);

// ============================================================================
// FUNCTION DECLARATIONS - SENSORS
// ============================================================================

/**
 * @brief Initialize all sensors (Hall, current, thermal, limit switches)
 */
void sensors_init(void);

/**
 * @brief Read motor current
 * @param motor_id Motor ID
 * @return Current in milliamps
 */
float read_current(motor_id_t motor_id);

/**
 * @brief Read motor end red-yellow wire sensors
 * @param motor_id Motor ID
 * @param red_wire Pointer to store red wire status (true = alarm)
 * @param yellow_wire Pointer to store yellow wire status (true = alarm)
 * @return Calculated temperature in Celsius, or -1 if no alarm
 */
float read_motor_end_sensors(motor_id_t motor_id, bool* red_wire, bool* yellow_wire);

/**
 * @brief Check if limit switch is triggered
 * @param motor_id Motor ID
 * @param top true for top limit, false for bottom limit
 * @return true if limit switch is triggered
 */
bool check_limit_switch(motor_id_t motor_id, bool top);

/**
 * @brief Check all safety conditions and stop motors if needed
 */
void safety_check(void);

/**
 * @brief Update d height from hall sensor counts
 */
void update_height(void);

/**
 * @brief Get current d height
 * @return Height in millimeters
 */
float get_current_height(void);

/**
 * @brief Calibrate height at current position
 * @param known_height_mm Known height in millimeters
 */
void calibrate_height(float known_height_mm);

// ============================================================================
// FUNCTION DECLARATIONS - SYNCHRONIZATION
// ============================================================================

/**
 * @brief Synchronize both motors to prevent racking
 * @param base_speed Base speed for both motors
 */
void sync_motors(int16_t base_speed);

/**
 * @brief Get motor state (thread-safe)
 * @param motor_id Motor ID
 * @param state Pointer to state structure to fill
 */
void get_motor_state(motor_id_t motor_id, motor_state_t* state);

/**
 * @brief Get d state (thread-safe)
 * @param state Pointer to d state structure to fill
 */
void get_d_state(d_state_t* state);

// ============================================================================
// FUNCTION DECLARATIONS - FREERTOS TASKS
// ============================================================================

/**
 * @brief Motor control task - handles command queue and speed ramping
 */
void task_motor_control(void *param);

/**
 * @brief Current monitoring task - checks for over-current conditions
 */
void task_current_monitor(void *param);

/**
 * @brief Hall sensor monitoring task - calculates RPM and updates position
 */
void task_hall_monitor(void *param);

/**
 * @brief Command handler task - processes high-level commands
 */
void task_command_handler(void *param);

/**
 * @brief Telemetry task - logs motor status via serial
 */
void task_telemetry(void *param);

// ============================================================================
// FUNCTION DECLARATIONS - ISR HANDLERS
// ============================================================================

/**
 * @brief Hall sensor ISR for Motor 1
 */
void IRAM_ATTR motor1_hall_isr(void* arg);

/**
 * @brief Hall sensor ISR for Motor 2
 */
void IRAM_ATTR motor2_hall_isr(void* arg);

/**
 * @brief Limit switch ISR for emergency stop
 */
void IRAM_ATTR limit_switch_isr(void* arg);

/**
 * @brief Thermal sensor ISR for emergency stop
 */
void IRAM_ATTR thermal_isr(void* arg);

// ============================================================================
// HELPER MACROS
// ============================================================================

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#define ABS(val) ((val) < 0 ? -(val) : (val))

#ifdef __cplusplus
}
#endif
