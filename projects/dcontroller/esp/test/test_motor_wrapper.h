// Test wrapper that provides all motor types without ESP-IDF dependencies
// Include this file instead of motor/motor.h in tests

#pragma once

// Standard headers
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// First include all mocks
#include "mocks/test_mocks.h"

// Unity testing framework
#include <unity.h>

// ============================================================================
// Type Definitions (copied from motor.h to avoid ESP-IDF includes)
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
    int16_t speed;
    motor_id_t motor_id;
} motor_command_t;

typedef struct {
    int16_t current_speed;
    int16_t target_speed;
    float current_ma;
    int32_t hall_count;
    bool over_current;
    bool thermal_shutdown;
    bool limit_triggered;
    bool red_wire_alarm;
    bool yellow_wire_alarm;
    float motor_end_temp;
} motor_state_t;

typedef struct {
    float height_mm;
    float target_height_mm;
    float mem1_height;
    float mem2_height;
    bool calibrated;
} d_state_t;

// ============================================================================
// Pin Configuration (copied from pin.h)
// ============================================================================

// Motor pins
#define M1_IN1    GPIO_NUM_25
#define M1_IN2    GPIO_NUM_26
#define M1_PWM_CH LEDC_CHANNEL_0
#define M2_IN1    GPIO_NUM_27
#define M2_IN2    GPIO_NUM_14
#define M2_PWM_CH LEDC_CHANNEL_1

// PWM configuration
#define PWM_FREQ      20000
#define PWM_RES       LEDC_TIMER_8_BIT
#define PWM_MAX       255
#define PWM_TIMER     LEDC_TIMER_0
#define PWM_MODE      LEDC_LOW_SPEED_MODE

// Hall sensors
#define HALL_M1       GPIO_NUM_34
#define HALL_M2       GPIO_NUM_35

// Limit switches
#define LIMIT_TOP_M1  GPIO_NUM_18
#define LIMIT_BOT_M1  GPIO_NUM_19
#define LIMIT_TOP_M2  GPIO_NUM_20
#define LIMIT_BOT_M2  GPIO_NUM_21

// Motor end sensors
#define M1_RED_PIN    GPIO_NUM_22
#define M1_YELLOW_PIN GPIO_NUM_23
#define M2_RED_PIN    GPIO_NUM_0
#define M2_YELLOW_PIN GPIO_NUM_2

#define RED_WIRE_LOGIC        0
#define YELLOW_WIRE_LOGIC     0

enum WireFunction {
    WIRE_UNKNOWN = 0,
    WIRE_THERMAL_CUTOFF,
    WIRE_THERMAL_WARNING,
    WIRE_TACHOMETER,
    WIRE_ENDSTOP_TOP,
    WIRE_ENDSTOP_BOTTOM
};

#define RED_WIRE_FUNCTION     WIRE_THERMAL_CUTOFF
#define YELLOW_WIRE_FUNCTION  WIRE_THERMAL_WARNING
#define THERMAL_CUTOFF_TEMP_C     80.0f
#define THERMAL_WARNING_TEMP_C    70.0f

// Current sensors
#define CUR_M1        ADC_CHANNEL_4
#define CUR_M2        ADC_CHANNEL_5
#define ADC_ATTEN     ADC_ATTEN_DB_12
#define ADC_WIDTH     ADC_BITWIDTH_12
#define ADC_VREF      3300
#define ACS712_OFFSET 1650
#define ACS712_SENS   66

// Buttons
#define BTN_UP        GPIO_NUM_4
#define BTN_DOWN      GPIO_NUM_16
#define BTN_M1        GPIO_NUM_17
#define BTN_M2        GPIO_NUM_5
#define DEBOUNCE_MS   50

// Display
#define I2C_SDA       GPIO_NUM_21
#define I2C_SCL       GPIO_NUM_22
#define OLED_ADDR     0x3C
#define OLED_WIDTH    128
#define OLED_HEIGHT   64

// Motion parameters
#define MOTOR_SPEED       200
#define CURRENT_LIMIT_A   2.8f
#define CURRENT_LIMIT_MA  2800
#define STALL_TIMEOUT_MS  300
#define SYNC_KP           0.5f

// Height calibration
#define PULSES_PER_MM     5.0f
#define MAX_HEIGHT_MM     500.0f
#define MIN_HEIGHT_MM     0.0f

// NVS keys
#define NVS_NAMESPACE     "d"
#define NVS_KEY_HEIGHT    "height"
#define NVS_KEY_M1_POS    "mem1"
#define NVS_KEY_M2_POS    "mem2"
#define NVS_KEY_CALIB     "calibrated"

// ============================================================================
// External Variables
// ============================================================================

extern d_state_t d_state;
extern motion_state_t motion_state;
extern motor_state_t motor_states[MOTOR_COUNT];
extern QueueHandle_t motor_command_queue;
extern SemaphoreHandle_t motor_state_mutex;
extern SemaphoreHandle_t d_state_mutex;

// Hall counters for test access
extern int32_t hall_counters[MOTOR_COUNT];
extern uint32_t last_hall_time_ms[MOTOR_COUNT];

// ============================================================================
// Function Declarations - MOTOR CONTROL
// ============================================================================

void motor_init(void);
void set_motor_speed(motor_id_t motor_id, int16_t speed);
void stop_motors(void);
void move_up(void);
void move_down(void);
bool move_to_height(float target_height_mm, uint32_t timeout_ms);

// ============================================================================
// Function Declarations - SENSORS
// ============================================================================

void sensors_init(void);
float read_current(motor_id_t motor_id);
float read_motor_end_sensors(motor_id_t motor_id, bool* red_wire, bool* yellow_wire);
bool check_limit_switch(motor_id_t motor_id, bool top);
void safety_check(void);
void update_height(void);
float get_current_height(void);
void calibrate_height(float known_height_mm);

// ============================================================================
// Function Declarations - SYNCHRONIZATION
// ============================================================================

void sync_motors(int16_t base_speed);
void get_motor_state(motor_id_t motor_id, motor_state_t* state);
void get_d_state(d_state_t* state);

// ============================================================================
// Function Declarations - DISPLAY
// ============================================================================

void storage_init(void);
void i2c_init(void);
void display_init(void);
void wifi_init(void);
void webserver_init(void);
void display_update(void);
void display_error(const char* msg);
void display_height(float height_mm);
void display_motor_status(const motor_state_t* m1, const motor_state_t* m2);
void buttons_init(void);
void buttons_poll(void);
bool button_was_pressed(uint8_t button_id);
float storage_load_height(const char* key);
esp_err_t storage_save_height(const char* key, float height);
int build_status_json(char* buf, size_t bufsize);
int build_motor_json(char* buf, size_t bufsize, const motor_state_t* states);

// ============================================================================
// HELPER MACROS
// ============================================================================

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#define ABS(val) ((val) < 0 ? -(val) : (val))
