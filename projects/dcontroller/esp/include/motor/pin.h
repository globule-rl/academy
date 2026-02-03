#pragma once

// Motor driver pins
#define M1_IN1 25
#define M1_IN2 26
#define M2_IN1 27
#define M2_IN2 14

// PWM channels
#define PWM_CH_M1 0
#define PWM_CH_M2 1

// Hall sensors
#define HALL_M1 34
#define HALL_M2 35

// PWM
#define PWM_FREQ 20000
#define PWM_RES  8
#define PWM_MAX  255

// Motion
#define MOTOR_SPEED 200
#define CURRENT_LIMIT 2.8   // amps
#define STALL_TIMEOUT 300   // ms without hall pulses

// Height calibration
#define PULSES_PER_MM 5.0   // adjust to your gearbox + hall sensor
#define MAX_HEIGHT_MM 500
#define MIN_HEIGHT_MM 0

// EEPROM / Preferences keys
#define PREF_NAMESPACE "d"
#define PREF_H1 "height1"
#define PREF_H2 "height2"

// Current sensors (ADC)
#define CUR_M1 32
#define CUR_M2 33

// Buttons
#define BTN_UP   4
#define BTN_DOWN 16
#define BTN_M1   17
#define BTN_M2   5
#define DEBOUNCE_DELAY 50     // ms

// I2C Display
#define SDA_PIN 21
#define SCL_PIN 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1