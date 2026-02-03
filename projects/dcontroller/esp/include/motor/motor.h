#pragma once

#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/idf_additions.h>

volatile long hallCountM1 = 0;
volatile long hallCountM2 = 0;

// Height
float heightMM = 0;

// Timing
unsigned long lastHallTime = 0;

// Motion state
enum Motion { STOPPED, MOVING_UP, MOVING_DOWN };
Motion motion = STOPPED;

// Motor IDs
#define MOTOR_UP 0
#define MOTOR_DOWN 1

// Command structure
struct MotorCommand {
    int speed;      // -255 to 255
    uint8_t motor_id;
};

// State structure
struct MotorState {
    int current_speed;
    int target_speed;
    float current_ma;
    int hall_count;
    bool over_current;
};

// External declarations
extern QueueHandle_t motorCommandQueue;
extern SemaphoreHandle_t motorStateMutex;
MotorState motor_states[2] = {
    {0, 0, 0, 0, false},
    {0, 0, 0, 0, false}
};

volatile int hall_counters[2] = {0, 0};

// Function declarations
void set_motor_speed(uint8_t motor_id, int speed);
float read_current(uint8_t motor_id);
void check_over_current();
void get_motor_state(uint8_t motor_id, MotorState* state);

// Inline helper for safe state reading
inline void get_motor_state(uint8_t motor_id, MotorState* state) {
    if (motor_id > 1 || !state) return;
    xSemaphoreTake(motorStateMutex, portMAX_DELAY);
    *state = motor_states[motor_id];
    xSemaphoreGive(motorStateMutex);
}

xTaskCreatePinnedToCore(
    task_motor_control, "Motor", 2048, NULL, 3, NULL, 0);
xTaskCreatePinnedToCore(
    task_current_monitor, "CurrentMon", 2048, NULL, 3, NULL, 0);
xTaskCreatePinnedToCore(
    task_hall_monitor, "HallMon", 2048, NULL, 2, NULL, 1);
xTaskCreatePinnedToCore(
    task_command_handler, "CmdHandler", 2048, NULL, 2, NULL, 1);
xTaskCreatePinnedToCore(
    task_telemetry, "Telemetry", 2048, NULL, 1, NULL, 1);


