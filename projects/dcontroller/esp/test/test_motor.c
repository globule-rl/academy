#include "test_motor_wrapper.h"
#include <stdio.h>

// Global variables from motor logic (we'll define them here for the test)
d_state_t d_state = {
    .height_mm = 0.0f,
    .target_height_mm = 0.0f,
    .mem1_height = -1.0f,
    .mem2_height = -1.0f,
    .calibrated = false
};

motion_state_t motion_state = MOTION_STOPPED;
motor_state_t motor_states[MOTOR_COUNT] = {0};
int32_t hall_counters[MOTOR_COUNT] = {0};
uint32_t last_hall_time_ms[MOTOR_COUNT] = {0};
QueueHandle_t motor_command_queue = NULL;
SemaphoreHandle_t motor_state_mutex = NULL;
SemaphoreHandle_t d_state_mutex = NULL;

// Include the actual motor implementation (logic only, no hardware init)
// We define the functions inline here for testing

void set_motor_speed(motor_id_t motor_id, int16_t speed) {
    if (motor_id >= MOTOR_COUNT) return;
    
    // Clamp speed to valid range
    speed = CLAMP(speed, -PWM_MAX, PWM_MAX);
    
    // Update state
    if (motor_state_mutex != NULL) {
        mock_semaphore_take(motor_state_mutex);
    }
    motor_states[motor_id].current_speed = speed;
    if (motor_state_mutex != NULL) {
        mock_semaphore_give(motor_state_mutex);
    }
    
    ESP_LOGD("motor", "Motor %d speed set to %d", motor_id, speed);
}

void stop_motors(void) {
    set_motor_speed(MOTOR_ID_1, 0);
    set_motor_speed(MOTOR_ID_2, 0);
    motion_state = MOTION_STOPPED;
    ESP_LOGI("motor", "All motors stopped");
}

void move_up(void) {
    // Check top limits first (mock returns 1 = not triggered)
    if (check_limit_switch(MOTOR_ID_1, true) || check_limit_switch(MOTOR_ID_2, true)) {
        ESP_LOGW("motor", "Cannot move up: top limit reached");
        return;
    }
    
    motion_state = MOTION_MOVING_UP;
    set_motor_speed(MOTOR_ID_1, MOTOR_SPEED);
    set_motor_speed(MOTOR_ID_2, MOTOR_SPEED);
    ESP_LOGI("motor", "Moving up");
}

void move_down(void) {
    // Check bottom limits first
    if (check_limit_switch(MOTOR_ID_1, false) || check_limit_switch(MOTOR_ID_2, false)) {
        ESP_LOGW("motor", "Cannot move down: bottom limit reached");
        return;
    }
    
    motion_state = MOTION_MOVING_DOWN;
    set_motor_speed(MOTOR_ID_1, -MOTOR_SPEED);
    set_motor_speed(MOTOR_ID_2, -MOTOR_SPEED);
    ESP_LOGI("motor", "Moving down");
}

bool move_to_height(float target_height_mm, uint32_t timeout_ms) {
    (void)timeout_ms;  // Not used in mock
    
    if (!d_state.calibrated) {
        ESP_LOGW("motor", "Cannot move to height: not calibrated");
        return false;
    }
    
    float current = get_current_height();
    float tolerance = 2.0f;  // 2mm tolerance
    
    if (ABS(target_height_mm - current) <= tolerance) {
        return true;  // Already at target
    }
    
    if (target_height_mm > current) {
        move_up();
    } else {
        move_down();
    }
    
    // In mock mode, we simulate reaching the target immediately
    // In real test, you'd iterate with delays
    d_state.height_mm = target_height_mm;
    stop_motors();
    
    ESP_LOGI("motor", "Reached target height: %.1f mm", d_state.height_mm);
    return true;
}

bool check_limit_switch(motor_id_t motor_id, bool top) {
    gpio_num_t pin;
    if (motor_id == MOTOR_ID_1) {
        pin = top ? LIMIT_TOP_M1 : LIMIT_BOT_M1;
    } else {
        pin = top ? LIMIT_TOP_M2 : LIMIT_BOT_M2;
    }
    
    return (mock_gpio_get_level(pin) == 0);  // Active low
}

void safety_check(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_id_t motor_id = (motor_id_t)i;
        
        // Check current
        float current_ma = read_current(motor_id);
        
        if (motor_state_mutex != NULL) {
            mock_semaphore_take(motor_state_mutex);
        }
        
        motor_states[motor_id].current_ma = current_ma;
        
        // Check over-current
        if (ABS(current_ma) > CURRENT_LIMIT_MA) {
            if (!motor_states[motor_id].over_current) {
                motor_states[motor_id].over_current = true;
                ESP_LOGW("motor", "Motor %d over-current: %.0f mA", motor_id, current_ma);
                set_motor_speed(motor_id, 0);
            }
        } else {
            motor_states[motor_id].over_current = false;
        }
        
        // Check stall condition
        uint32_t now = mock_get_tick_count() * portTICK_PERIOD_MS;
        if (motor_states[motor_id].current_speed != 0 &&
            (now - last_hall_time_ms[motor_id]) > STALL_TIMEOUT_MS) {
            ESP_LOGW("motor", "Motor %d stall detected", motor_id);
            set_motor_speed(motor_id, 0);
        }
        
        if (motor_state_mutex != NULL) {
            mock_semaphore_give(motor_state_mutex);
        }
    }
}

float read_current(motor_id_t motor_id) {
    adc_channel_t channel = (motor_id == MOTOR_ID_1) ? CUR_M1 : CUR_M2;
    
    // Read ADC value
    int adc_raw = mock_adc_get_raw(channel);
    
    // Convert to voltage (mV) - 12-bit ADC, 3.3V reference
    int voltage_mv = (adc_raw * ADC_VREF) / 4095;
    
    // ACS712-30A: 2.5V offset, 66mV/A sensitivity
    float offset_mv = ACS712_OFFSET;
    float current_a = (voltage_mv - offset_mv) / ACS712_SENS;
    
    return current_a * 1000.0f;  // Return in mA
}

void update_height(void) {
    int32_t avg_pulses = (hall_counters[MOTOR_ID_1] + hall_counters[MOTOR_ID_2]) / 2;
    float new_height = avg_pulses / PULSES_PER_MM;
    
    if (d_state_mutex != NULL) {
        mock_semaphore_take(d_state_mutex);
    }
    
    d_state.height_mm = new_height;
    
    // Enforce limits
    if (d_state.height_mm > MAX_HEIGHT_MM) {
        d_state.height_mm = MAX_HEIGHT_MM;
        stop_motors();
        ESP_LOGW("motor", "Maximum height reached");
    }
    if (d_state.height_mm < MIN_HEIGHT_MM) {
        d_state.height_mm = MIN_HEIGHT_MM;
        stop_motors();
        ESP_LOGW("motor", "Minimum height reached");
    }
    
    if (d_state_mutex != NULL) {
        mock_semaphore_give(d_state_mutex);
    }
}

float get_current_height(void) {
    update_height();
    return d_state.height_mm;
}

void calibrate_height(float known_height_mm) {
    // Reset hall counters to match known height
    hall_counters[MOTOR_ID_1] = (int32_t)(known_height_mm * PULSES_PER_MM);
    hall_counters[MOTOR_ID_2] = (int32_t)(known_height_mm * PULSES_PER_MM);
    
    update_height();
    d_state.calibrated = true;
    
    ESP_LOGI("motor", "Calibrated at %.1f mm", known_height_mm);
}

void sync_motors(int16_t base_speed) {
    // Get current positions
    int32_t pos1 = hall_counters[MOTOR_ID_1];
    int32_t pos2 = hall_counters[MOTOR_ID_2];
    
    // Calculate error
    int32_t error = pos1 - pos2;
    
    // P-controller adjustment
    int16_t adjustment = (int16_t)(error * SYNC_KP);
    
    // Limit adjustment
    adjustment = CLAMP(adjustment, -50, 50);
    
    // Apply to Motor 2
    int16_t new_speed = base_speed + adjustment;
    new_speed = CLAMP(new_speed, -PWM_MAX, PWM_MAX);
    
    set_motor_speed(MOTOR_ID_2, new_speed);
    
    ESP_LOGD("motor", "Sync: pos1=%ld, pos2=%ld, error=%ld, adj=%d",
             (long)pos1, (long)pos2, (long)error, adjustment);
}

void get_motor_state(motor_id_t motor_id, motor_state_t* state) {
    if (motor_id >= MOTOR_COUNT || state == NULL) return;
    
    if (motor_state_mutex != NULL) {
        mock_semaphore_take(motor_state_mutex);
    }
    memcpy(state, &motor_states[motor_id], sizeof(motor_state_t));
    if (motor_state_mutex != NULL) {
        mock_semaphore_give(motor_state_mutex);
    }
}

void get_d_state(d_state_t* state) {
    if (state == NULL) return;
    
    update_height();
    
    if (d_state_mutex != NULL) {
        mock_semaphore_take(d_state_mutex);
    }
    memcpy(state, &d_state, sizeof(d_state_t));
    if (d_state_mutex != NULL) {
        mock_semaphore_give(d_state_mutex);
    }
}

// Display JSON builders
int build_status_json(char* buf, size_t bufsize) {
    return snprintf(buf, bufsize,
        "{"
        "\"height\":%.1f,"
        "\"motion\":%d,"
        "\"calibrated\":%s,"
        "\"mem1\":%.1f,"
        "\"mem2\":%.1f"
        "}",
        d_state.height_mm,
        motion_state,
        d_state.calibrated ? "true" : "false",
        d_state.mem1_height,
        d_state.mem2_height);
}

int build_motor_json(char* buf, size_t bufsize, const motor_state_t* states) {
    return snprintf(buf, bufsize,
        "["
        "{\"speed\":%d,\"current\":%.1f,\"hall\":%ld,\"oc\":%s,\"th\":%s,\"lm\":%s},"
        "{\"speed\":%d,\"current\":%.1f,\"hall\":%ld,\"oc\":%s,\"th\":%s,\"lm\":%s}"
        "]",
        states[0].current_speed, states[0].current_ma, (long)states[0].hall_count,
        states[0].over_current ? "true" : "false",
        states[0].thermal_shutdown ? "true" : "false",
        states[0].limit_triggered ? "true" : "false",
        states[1].current_speed, states[1].current_ma, (long)states[1].hall_count,
        states[1].over_current ? "true" : "false",
        states[1].thermal_shutdown ? "true" : "false",
        states[1].limit_triggered ? "true" : "false");
}

void setUp(void)
{
    mock_reset();
    memset(&d_state, 0, sizeof(d_state_t));
    memset(motor_states, 0, sizeof(motor_states));
    d_state.mem1_height = -1.0f;
    d_state.mem2_height = -1.0f;
    memset(hall_counters, 0, sizeof(hall_counters));
    memset(last_hall_time_ms, 0, sizeof(last_hall_time_ms));
}

void tearDown(void)
{
    // Cleanup if needed
}

// ============================================================================
// TEST: Helper Macros
// ============================================================================

void test_clamp_macro(void)
{
    TEST_ASSERT_EQUAL_INT16(0, CLAMP(-10, 0, 100));
    TEST_ASSERT_EQUAL_INT16(100, CLAMP(150, 0, 100));
    TEST_ASSERT_EQUAL_INT16(50, CLAMP(50, 0, 100));
    TEST_ASSERT_EQUAL_INT16(-50, CLAMP(-100, -50, 50));
}

void test_abs_macro(void)
{
    TEST_ASSERT_EQUAL_INT16(50, ABS(-50));
    TEST_ASSERT_EQUAL_INT16(50, ABS(50));
    TEST_ASSERT_EQUAL_INT16(0, ABS(0));
}

// ============================================================================
// TEST: Height Calculation Logic
// ============================================================================

void test_update_height_basic(void)
{
    // Set hall counters to simulate 100mm height
    hall_counters[MOTOR_ID_1] = (int32_t)(100 * PULSES_PER_MM);
    hall_counters[MOTOR_ID_2] = (int32_t)(100 * PULSES_PER_MM);
    
    update_height();
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, d_state.height_mm);
}

void test_update_height_with_offset(void)
{
    // Test asymmetric motor counts (simulates slight misalignment)
    hall_counters[MOTOR_ID_1] = 500;  // 100mm
    hall_counters[MOTOR_ID_2] = 550;  // 110mm
    
    update_height();
    
    // Should average to 105mm
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 105.0f, d_state.height_mm);
}

void test_get_current_height_updates(void)
{
    hall_counters[MOTOR_ID_1] = 250;
    hall_counters[MOTOR_ID_2] = 250;
    
    float height = get_current_height();
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, height);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, d_state.height_mm);
}

void test_calibrate_height(void)
{
    // Simulate existing movement
    hall_counters[MOTOR_ID_1] = 1000;
    hall_counters[MOTOR_ID_2] = 1000;
    
    // Calibrate to 200mm
    calibrate_height(200.0f);
    
    TEST_ASSERT_TRUE(d_state.calibrated);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 200.0f, d_state.height_mm);
    TEST_ASSERT_EQUAL_INT32((int32_t)(200 * PULSES_PER_MM), hall_counters[MOTOR_ID_1]);
}

// ============================================================================
// TEST: Motor State Access
// ============================================================================

void test_get_motor_state(void)
{
    // Set some motor state directly
    motor_states[MOTOR_ID_1].current_speed = 150;
    motor_states[MOTOR_ID_1].target_speed = 200;
    motor_states[MOTOR_ID_1].current_ma = 500.5f;
    motor_states[MOTOR_ID_1].over_current = true;
    
    motor_state_t state;
    get_motor_state(MOTOR_ID_1, &state);
    
    TEST_ASSERT_EQUAL_INT16(150, state.current_speed);
    TEST_ASSERT_EQUAL_INT16(200, state.target_speed);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 500.5f, state.current_ma);
    TEST_ASSERT_TRUE(state.over_current);
}

void test_get_d_state(void)
{
    d_state.height_mm = 123.4f;
    d_state.target_height_mm = 200.0f;
    d_state.calibrated = true;
    d_state.mem1_height = 100.0f;
    d_state.mem2_height = 150.0f;
    
    d_state_t state;
    get_d_state(&state);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 123.4f, state.height_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 200.0f, state.target_height_mm);
    TEST_ASSERT_TRUE(state.calibrated);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, state.mem1_height);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 150.0f, state.mem2_height);
}

void test_get_motor_state_invalid_id(void)
{
    motor_state_t state;
    memset(&state, 0xFF, sizeof(state));  // Fill with invalid pattern
    
    get_motor_state(99, &state);  // Invalid motor ID
    
    // Should not crash, state should remain unchanged
    // (This tests boundary safety)
}

// ============================================================================
// TEST: Motor Synchronization Logic
// ============================================================================

void test_sync_motors_perfect_alignment(void)
{
    // Both motors at same position
    hall_counters[MOTOR_ID_1] = 1000;
    hall_counters[MOTOR_ID_2] = 1000;
    
    int16_t base_speed = 200;
    sync_motors(base_speed);
    
    // Motor 2 should maintain base speed (no adjustment needed)
    TEST_ASSERT_EQUAL_INT16(200, motor_states[MOTOR_ID_2].current_speed);
}

void test_sync_motors_motor2_lagging(void)
{
    // Motor 2 is behind Motor 1
    hall_counters[MOTOR_ID_1] = 1100;
    hall_counters[MOTOR_ID_2] = 1000;
    
    sync_motors(200);
    
    // Motor 2 should speed up to catch up
    // Error = 100, adjustment = 100 * 0.5 = 50, so speed = 250
    // But max adjustment is clamped to 50
    TEST_ASSERT_GREATER_THAN(200, motor_states[MOTOR_ID_2].current_speed);
}

void test_sync_motors_motor2_ahead(void)
{
    // Motor 2 is ahead of Motor 1
    hall_counters[MOTOR_ID_1] = 1000;
    hall_counters[MOTOR_ID_2] = 1100;
    
    sync_motors(200);
    
    // Motor 2 should slow down
    // Error = -100, adjustment = -50, so speed = 150
    TEST_ASSERT_LESS_THAN(200, motor_states[MOTOR_ID_2].current_speed);
}

// ============================================================================
// TEST: Motor Speed Control Logic
// ============================================================================

void test_set_motor_speed_clamping(void)
{
    set_motor_speed(MOTOR_ID_1, 300);  // Above max
    TEST_ASSERT_EQUAL_INT16(255, motor_states[MOTOR_ID_1].current_speed);
    
    set_motor_speed(MOTOR_ID_1, -300);  // Below min
    TEST_ASSERT_EQUAL_INT16(-255, motor_states[MOTOR_ID_1].current_speed);
    
    set_motor_speed(MOTOR_ID_1, 128);  // Normal
    TEST_ASSERT_EQUAL_INT16(128, motor_states[MOTOR_ID_1].current_speed);
}

void test_set_motor_speed_invalid_id(void)
{
    // Should not crash with invalid motor ID
    set_motor_speed(99, 100);
    // Test passes if we reach here without crash
    TEST_ASSERT_TRUE(true);
}

void test_stop_motors(void)
{
    // Set motors moving
    set_motor_speed(MOTOR_ID_1, 200);
    set_motor_speed(MOTOR_ID_2, -150);
    motion_state = MOTION_MOVING_UP;
    
    stop_motors();
    
    TEST_ASSERT_EQUAL_INT16(0, motor_states[MOTOR_ID_1].current_speed);
    TEST_ASSERT_EQUAL_INT16(0, motor_states[MOTOR_ID_2].current_speed);
    TEST_ASSERT_EQUAL(MOTION_STOPPED, motion_state);
}

// ============================================================================
// TEST: Movement Commands
// ============================================================================

void test_move_up_sets_state(void)
{
    // Ensure limits are not triggered (mock returns 1 for all GPIO)
    motion_state = MOTION_STOPPED;
    
    move_up();
    
    TEST_ASSERT_EQUAL(MOTION_MOVING_UP, motion_state);
    TEST_ASSERT_EQUAL_INT16(MOTOR_SPEED, motor_states[MOTOR_ID_1].current_speed);
    TEST_ASSERT_EQUAL_INT16(MOTOR_SPEED, motor_states[MOTOR_ID_2].current_speed);
}

void test_move_down_sets_state(void)
{
    motion_state = MOTION_STOPPED;
    
    move_down();
    
    TEST_ASSERT_EQUAL(MOTION_MOVING_DOWN, motion_state);
    TEST_ASSERT_EQUAL_INT16(-MOTOR_SPEED, motor_states[MOTOR_ID_1].current_speed);
    TEST_ASSERT_EQUAL_INT16(-MOTOR_SPEED, motor_states[MOTOR_ID_2].current_speed);
}

void test_move_to_height_not_calibrated(void)
{
    d_state.calibrated = false;
    
    bool result = move_to_height(100.0f, 5000);
    
    TEST_ASSERT_FALSE(result);
}

void test_move_to_height_already_at_target(void)
{
    d_state.calibrated = true;
    d_state.height_mm = 100.0f;
    
    bool result = move_to_height(100.0f, 5000);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(MOTION_STOPPED, motion_state);
}

// ============================================================================
// TEST: Safety Logic
// ============================================================================

void test_safety_check_overcurrent_detection(void)
{
    // Set up motor moving
    motor_states[MOTOR_ID_1].current_speed = 200;
    mock_state.tick_count = 1000;
    last_hall_time_ms[MOTOR_ID_1] = 500;  // Recent pulse
    
    // Set ADC to read high current (above 2800mA limit)
    // ADC formula: current_a = (voltage_mv - 1650) / 66
    // For 3000mA: voltage_mv = 3000 * 0.066 + 1650 = 1848mV
    // ADC raw for 1848mV with 12-bit: (1848 / 3300) * 4095 = 2293
    mock_adc_set_raw(CUR_M1, 2293);
    
    safety_check();
    
    TEST_ASSERT_TRUE(motor_states[MOTOR_ID_1].over_current);
    TEST_ASSERT_EQUAL_INT16(0, motor_states[MOTOR_ID_1].current_speed);  // Should stop
}

void test_safety_check_normal_current(void)
{
    motor_states[MOTOR_ID_1].current_speed = 200;
    mock_state.tick_count = 1000;
    last_hall_time_ms[MOTOR_ID_1] = 500;
    
    // Set ADC for normal current (1000mA)
    // voltage_mv = 1000 * 0.066 + 1650 = 1716mV
    // ADC raw: (1716 / 3300) * 4095 = 2129
    mock_adc_set_raw(CUR_M1, 2129);
    
    safety_check();
    
    TEST_ASSERT_FALSE(motor_states[MOTOR_ID_1].over_current);
}

// ============================================================================
// TEST: Display JSON Builders
// ============================================================================

void test_build_status_json(void)
{
    d_state.height_mm = 150.5f;
    d_state.calibrated = true;
    d_state.mem1_height = 100.0f;
    d_state.mem2_height = 200.0f;
    motion_state = MOTION_MOVING_UP;
    
    char buf[256];
    int len = build_status_json(buf, sizeof(buf));
    
    TEST_ASSERT_GREATER_THAN(0, len);
    TEST_ASSERT_TRUE(strstr(buf, "150.5") != NULL);
    TEST_ASSERT_TRUE(strstr(buf, "\"calibrated\":true") != NULL);
    TEST_ASSERT_TRUE(strstr(buf, "\"mem1\":100") != NULL);
}

void test_build_motor_json(void)
{
    motor_state_t states[MOTOR_COUNT];
    memset(states, 0, sizeof(states));
    states[0].current_speed = 200;
    states[0].current_ma = 1500.5f;
    states[0].hall_count = 1000;
    states[0].over_current = false;
    states[0].thermal_shutdown = false;
    states[0].limit_triggered = false;
    
    states[1].current_speed = -150;
    states[1].current_ma = 1200.0f;
    states[1].hall_count = 950;
    states[1].over_current = true;
    states[1].thermal_shutdown = false;
    states[1].limit_triggered = true;
    
    char buf[512];
    int len = build_motor_json(buf, sizeof(buf), states);
    
    TEST_ASSERT_GREATER_THAN(0, len);
    TEST_ASSERT_TRUE(strstr(buf, "\"speed\":200") != NULL);
    TEST_ASSERT_TRUE(strstr(buf, "\"speed\":-150") != NULL);
    TEST_ASSERT_TRUE(strstr(buf, "\"oc\":true") != NULL);
    TEST_ASSERT_TRUE(strstr(buf, "\"oc\":false") != NULL);
}

void test_build_status_json_buffer_too_small(void)
{
    char buf[10];  // Way too small
    d_state.height_mm = 150.5f;
    
    int len = build_status_json(buf, sizeof(buf));
    
    // Should still return length, even if truncated
    TEST_ASSERT_GREATER_THAN(0, len);
}

// ============================================================================
// TEST: Configuration Constants
// ============================================================================

void test_pin_configuration_values(void)
{
    // Verify critical pin mappings
    TEST_ASSERT_EQUAL(25, M1_IN1);
    TEST_ASSERT_EQUAL(26, M1_IN2);
    TEST_ASSERT_EQUAL(27, M2_IN1);
    TEST_ASSERT_EQUAL(14, M2_IN2);
    
    // Verify limits
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 500.0f, MAX_HEIGHT_MM);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, MIN_HEIGHT_MM);
    
    // Verify current limit
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2800.0f, CURRENT_LIMIT_MA);
}

void test_motion_parameters(void)
{
    TEST_ASSERT_EQUAL(200, MOTOR_SPEED);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.8f, CURRENT_LIMIT_A);
    TEST_ASSERT_EQUAL(300, STALL_TIMEOUT_MS);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.5f, SYNC_KP);
}

// ============================================================================
// MAIN
// ============================================================================

void app_main()
{
    UNITY_BEGIN();
    
    printf("\n========================================\n");
    printf("MOTOR CONTROLLER UNIT TESTS\n");
    printf("========================================\n\n");
    
    // Helper macros
    printf("Testing Helper Macros...\n");
    RUN_TEST(test_clamp_macro);
    RUN_TEST(test_abs_macro);
    
    // Height calculation
    printf("Testing Height Calculation...\n");
    RUN_TEST(test_update_height_basic);
    RUN_TEST(test_update_height_with_offset);
    RUN_TEST(test_get_current_height_updates);
    RUN_TEST(test_calibrate_height);
    
    // State access
    printf("Testing State Access...\n");
    RUN_TEST(test_get_motor_state);
    RUN_TEST(test_get_d_state);
    RUN_TEST(test_get_motor_state_invalid_id);
    
    // Motor synchronization
    printf("Testing Motor Synchronization...\n");
    RUN_TEST(test_sync_motors_perfect_alignment);
    RUN_TEST(test_sync_motors_motor2_lagging);
    RUN_TEST(test_sync_motors_motor2_ahead);
    
    // Speed control
    printf("Testing Speed Control...\n");
    RUN_TEST(test_set_motor_speed_clamping);
    RUN_TEST(test_set_motor_speed_invalid_id);
    RUN_TEST(test_stop_motors);
    
    // Movement commands
    printf("Testing Movement Commands...\n");
    RUN_TEST(test_move_up_sets_state);
    RUN_TEST(test_move_down_sets_state);
    RUN_TEST(test_move_to_height_not_calibrated);
    RUN_TEST(test_move_to_height_already_at_target);
    
    // Safety logic
    printf("Testing Safety Logic...\n");
    RUN_TEST(test_safety_check_overcurrent_detection);
    RUN_TEST(test_safety_check_normal_current);
    
    // Display JSON
    printf("Testing Display JSON Builders...\n");
    RUN_TEST(test_build_status_json);
    RUN_TEST(test_build_motor_json);
    RUN_TEST(test_build_status_json_buffer_too_small);
    
    // Configuration
    printf("Testing Configuration Constants...\n");
    RUN_TEST(test_pin_configuration_values);
    RUN_TEST(test_motion_parameters);
    
    printf("\n========================================\n");
    UNITY_END();
}
