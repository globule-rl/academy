#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ============================================================================
// MOTOR DRIVER PINS (BTS7960 / IBT-2)
// ============================================================================
// Motor 1 pins
#define M1_IN1    GPIO_NUM_25   // PWM pin for Motor 1
#define M1_IN2    GPIO_NUM_26   // Direction pin for Motor 1
#define M1_PWM_CH LEDC_CHANNEL_0

// Motor 2 pins  
#define M2_IN1    GPIO_NUM_27   // PWM pin for Motor 2
#define M2_IN2    GPIO_NUM_14   // Direction pin for Motor 2
#define M2_PWM_CH LEDC_CHANNEL_1

// ============================================================================
// PWM CONFIGURATION
// ============================================================================
#define PWM_FREQ      20000     // 20kHz PWM frequency
#define PWM_RES       LEDC_TIMER_8_BIT  // 8-bit resolution
#define PWM_MAX       255
#define PWM_TIMER     LEDC_TIMER_0
#define PWM_MODE      LEDC_LOW_SPEED_MODE

// ============================================================================
// HALL SENSORS (Position feedback)
// ============================================================================
#define HALL_M1       GPIO_NUM_34   // Hall sensor for Motor 1
#define HALL_M2       GPIO_NUM_35   // Hall sensor for Motor 2

// ============================================================================
// END-OF-TRAVEL SENSORS (Limit switches)
// ============================================================================
#define LIMIT_TOP_M1  GPIO_NUM_18   // Top limit switch Motor 1
#define LIMIT_BOT_M1  GPIO_NUM_19   // Bottom limit switch Motor 1
#define LIMIT_TOP_M2  GPIO_NUM_20   // Top limit switch Motor 2
#define LIMIT_BOT_M2  GPIO_NUM_21   // Bottom limit switch Motor 2

// ============================================================================
// MOTOR END SENSORS (Red-yellow wire pair at opposite end from hall sensors)
// Configuration: Each motor has TWO pins (red wire + yellow wire)
// Adjust these based on your actual sensor testing
// ============================================================================

// Motor 1 end sensors (red + yellow wire pair)
#define M1_RED_PIN    GPIO_NUM_22   // Red wire sensor
#define M1_YELLOW_PIN GPIO_NUM_23   // Yellow wire sensor

// Motor 2 end sensors (red + yellow wire pair)  
#define M2_RED_PIN    GPIO_NUM_0    // Red wire sensor
#define M2_YELLOW_PIN GPIO_NUM_2    // Yellow wire sensor

// Sensor behavior configuration (ADJUST AFTER TESTING):
#define RED_WIRE_LOGIC        0     // 0 = active LOW, 1 = active HIGH
#define YELLOW_WIRE_LOGIC     0     // 0 = active LOW, 1 = active HIGH

// Define what each wire does (ADJUST AFTER TESTING):
// Options: THERMAL_CUTOFF, THERMAL_WARNING, TACHOMETER, ENDSTOP, UNKNOWN
#define RED_WIRE_FUNCTION     THERMAL_CUTOFF    // Primary emergency thermal
#define YELLOW_WIRE_FUNCTION  THERMAL_WARNING   // Secondary warning/alarm

// Wire function enum for configuration
enum WireFunction {
    WIRE_UNKNOWN = 0,
    WIRE_THERMAL_CUTOFF,    // Emergency stop when overheating
    WIRE_THERMAL_WARNING,   // Warning before cutoff
    WIRE_TACHOMETER,        // Speed sensing (pulses)
    WIRE_ENDSTOP_TOP,       // Top end limit
    WIRE_ENDSTOP_BOTTOM     // Bottom end limit
};

// Thresholds (ADJUST AFTER TESTING)
#define THERMAL_CUTOFF_TEMP_C     80.0f   // Emergency stop temperature
#define THERMAL_WARNING_TEMP_C    70.0f   // Warning temperature

// ============================================================================
// CURRENT SENSORS (ACS712-30A)
// ============================================================================
#define CUR_M1        ADC1_CHANNEL_4   // GPIO32 - Motor 1 current
#define CUR_M2        ADC1_CHANNEL_5   // GPIO33 - Motor 2 current
#define ADC_ATTEN     ADC_ATTEN_DB_11  // 0-3.3V range
#define ADC_WIDTH     ADC_WIDTH_BIT_12 // 12-bit resolution
#define ADC_VREF      3300             // 3.3V in mV
#define ACS712_OFFSET 1650             // 2.5V offset in mV (0A point)
#define ACS712_SENS   66               // 66mV/A for 30A version

// ============================================================================
// USER INTERFACE (Buttons)
// ============================================================================
#define BTN_UP        GPIO_NUM_4
#define BTN_DOWN      GPIO_NUM_16
#define BTN_M1        GPIO_NUM_17    // Memory 1 button
#define BTN_M2        GPIO_NUM_5     // Memory 2 button
#define DEBOUNCE_MS   50             // Debounce time in milliseconds

// ============================================================================
// DISPLAY (I2C OLED)
// ============================================================================
#define I2C_SDA       GPIO_NUM_21
#define I2C_SCL       GPIO_NUM_22
#define I2C_FREQ      400000         // 400kHz I2C speed
#define OLED_ADDR     0x3C
#define OLED_WIDTH    128
#define OLED_HEIGHT   64

// ============================================================================
// MOTION PARAMETERS
// ============================================================================
#define MOTOR_SPEED       200     // Default PWM speed (0-255)
#define CURRENT_LIMIT_A   2.8f    // Current limit in Amps
#define CURRENT_LIMIT_MA  2800    // Current limit in mA
#define STALL_TIMEOUT_MS  300     // Stall detection timeout in ms
#define SYNC_KP           0.5f    // Proportional gain for motor synchronization

// ============================================================================
// HEIGHT CALIBRATION
// ============================================================================
#define PULSES_PER_MM     5.0f    // Hall pulses per millimeter
#define MAX_HEIGHT_MM     500.0f  // Maximum desk height
#define MIN_HEIGHT_MM     0.0f    // Minimum desk height

// ============================================================================
// NVS (Preferences) KEYS
// ============================================================================
#define NVS_NAMESPACE     "desk"
#define NVS_KEY_HEIGHT    "height"
#define NVS_KEY_M1_POS    "mem1"
#define NVS_KEY_M2_POS    "mem2"
#define NVS_KEY_CALIB     "calibrated"

// ============================================================================
// TASK CONFIGURATION
// ============================================================================
#define TASK_MOTOR_CTRL_STACK     4096
#define TASK_CURRENT_MON_STACK    4096
#define TASK_HALL_MON_STACK       4096
#define TASK_CMD_HANDLER_STACK    4096
#define TASK_TELEMETRY_STACK      4096
#define TASK_DISPLAY_STACK        4096
#define TASK_WEBSERVER_STACK      8192
#define TASK_BUTTON_STACK         4096

#define TASK_MOTOR_CTRL_PRIO      10
#define TASK_CURRENT_MON_PRIO     9
#define TASK_HALL_MON_PRIO        8
#define TASK_CMD_HANDLER_PRIO     7
#define TASK_TELEMETRY_PRIO       5
#define TASK_DISPLAY_PRIO         4
#define TASK_WEBSERVER_PRIO       3
#define TASK_BUTTON_PRIO          6

#define CORE_0                    0
#define CORE_1                    1
