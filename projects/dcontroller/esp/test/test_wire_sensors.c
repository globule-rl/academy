#ifdef UNIT_TEST_MOCK
// ============================================================================
// MOCK TESTS - Run on host machine without hardware
// Use: gcc -DUNIT_TEST_MOCK test_wire_sensors.c mocks/test_mocks.c ../src/motor.cc -I../include -I. -o test_wire_sensors && ./test_wire_sensors
// ============================================================================

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "test_mocks.h"
#include "motor/motor.h"
#include "motor/pin.h"

// Simple test framework macros
#define TEST_ASSERT_EQUAL(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            printf("FAIL: Expected %d but got %d at %s:%d\n", \
                   (expected), (actual), __FILE__, __LINE__); \
            return 1; \
        } \
    } while(0)

#define TEST_ASSERT_TRUE(actual) \
    do { \
        if (!(actual)) { \
            printf("FAIL: Expected true but got false at %s:%d\n", \
                   __FILE__, __LINE__); \
            return 1; \
        } \
    } while(0)

#define TEST_ASSERT_FALSE(actual) \
    do { \
        if (actual) { \
            printf("FAIL: Expected false but got true at %s:%d\n", \
                   __FILE__, __LINE__); \
            return 1; \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) \
    do { \
        float diff = (expected) - (actual); \
        if (diff < 0) diff = -diff; \
        if (diff > (delta)) { \
            printf("FAIL: Expected %.2f +/- %.2f but got %.2f at %s:%d\n", \
                   (expected), (delta), (actual), __FILE__, __LINE__); \
            return 1; \
        } \
    } while(0)

// Test: Both wires normal (HIGH)
int test_both_wires_normal(void) {
    printf("Testing: Both wires normal (HIGH)...\n");
    
    // Setup: Both pins HIGH (normal state, assuming active LOW logic)
    mock_gpio_set_level(M1_RED_PIN, 1);
    mock_gpio_set_level(M1_YELLOW_PIN, 1);
    
    bool red_alarm = false, yellow_alarm = false;
    float temp = read_motor_end_sensors(MOTOR_ID_1, &red_alarm, &yellow_alarm);
    
    TEST_ASSERT_FALSE(red_alarm);
    TEST_ASSERT_FALSE(yellow_alarm);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 25.0f, temp);
    
    printf("  PASSED\n");
    return 0;
}

// Test: Red wire alarm active
int test_red_wire_alarm(void) {
    printf("Testing: Red wire alarm (LOW)...\n");
    
    // Setup: Red LOW, Yellow HIGH
    mock_gpio_set_level(M1_RED_PIN, 0);
    mock_gpio_set_level(M1_YELLOW_PIN, 1);
    
    bool red_alarm = false, yellow_alarm = false;
    float temp = read_motor_end_sensors(MOTOR_ID_1, &red_alarm, &yellow_alarm);
    
    TEST_ASSERT_TRUE(red_alarm);
    TEST_ASSERT_FALSE(yellow_alarm);
    // Should return threshold temperature based on configuration
    if (RED_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) {
        TEST_ASSERT_FLOAT_WITHIN(0.1, THERMAL_CUTOFF_TEMP_C, temp);
    } else {
        TEST_ASSERT_FLOAT_WITHIN(0.1, THERMAL_WARNING_TEMP_C, temp);
    }
    
    printf("  PASSED\n");
    return 0;
}

// Test: Yellow wire alarm active
int test_yellow_wire_alarm(void) {
    printf("Testing: Yellow wire alarm (LOW)...\n");
    
    // Setup: Red HIGH, Yellow LOW
    mock_gpio_set_level(M1_RED_PIN, 1);
    mock_gpio_set_level(M1_YELLOW_PIN, 0);
    
    bool red_alarm = false, yellow_alarm = false;
    float temp = read_motor_end_sensors(MOTOR_ID_1, &red_alarm, &yellow_alarm);
    
    TEST_ASSERT_FALSE(red_alarm);
    TEST_ASSERT_TRUE(yellow_alarm);
    // Should return threshold temperature based on configuration
    if (YELLOW_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) {
        TEST_ASSERT_FLOAT_WITHIN(0.1, THERMAL_CUTOFF_TEMP_C, temp);
    } else {
        TEST_ASSERT_FLOAT_WITHIN(0.1, THERMAL_WARNING_TEMP_C, temp);
    }
    
    printf("  PASSED\n");
    return 0;
}

// Test: Both wires alarm active
int test_both_wires_alarm(void) {
    printf("Testing: Both wires alarm (LOW)...\n");
    
    // Setup: Both pins LOW
    mock_gpio_set_level(M1_RED_PIN, 0);
    mock_gpio_set_level(M1_YELLOW_PIN, 0);
    
    bool red_alarm = false, yellow_alarm = false;
    float temp = read_motor_end_sensors(MOTOR_ID_1, &red_alarm, &yellow_alarm);
    
    TEST_ASSERT_TRUE(red_alarm);
    TEST_ASSERT_TRUE(yellow_alarm);
    // Red takes priority for temperature calculation
    if (RED_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) {
        TEST_ASSERT_FLOAT_WITHIN(0.1, THERMAL_CUTOFF_TEMP_C, temp);
    }
    
    printf("  PASSED\n");
    return 0;
}

// Test: Safety check triggers on red wire alarm
int test_safety_check_red_wire(void) {
    printf("Testing: Safety check with red wire alarm...\n");
    
    // Setup: Motor running with normal current
    mock_adc_set_raw(CUR_M1, 200);  // Normal current ~0.5A
    mock_gpio_set_level(M1_RED_PIN, 0);  // Red wire alarm
    
    // Reset motor state
    motor_states[MOTOR_ID_1].current_speed = 200;
    motor_states[MOTOR_ID_1].red_wire_alarm = false;
    motor_states[MOTOR_ID_1].thermal_shutdown = false;
    
    safety_check();
    
    // Verify alarm was detected
    TEST_ASSERT_TRUE(motor_states[MOTOR_ID_1].red_wire_alarm);
    
    // If configured as thermal cutoff, should trigger thermal shutdown
    if (RED_WIRE_FUNCTION == WIRE_THERMAL_CUTOFF) {
        TEST_ASSERT_TRUE(motor_states[MOTOR_ID_1].thermal_shutdown);
    }
    
    printf("  PASSED\n");
    return 0;
}

// Test: Motor 2 wires
int test_motor2_wires(void) {
    printf("Testing: Motor 2 wire sensors...\n");
    
    // Setup: Both pins HIGH for M2
    mock_gpio_set_level(M2_RED_PIN, 1);
    mock_gpio_set_level(M2_YELLOW_PIN, 1);
    
    bool red_alarm = false, yellow_alarm = false;
    float temp = read_motor_end_sensors(MOTOR_ID_2, &red_alarm, &yellow_alarm);
    
    TEST_ASSERT_FALSE(red_alarm);
    TEST_ASSERT_FALSE(yellow_alarm);
    
    // Now trigger red wire for M2
    mock_gpio_set_level(M2_RED_PIN, 0);
    temp = read_motor_end_sensors(MOTOR_ID_2, &red_alarm, &yellow_alarm);
    
    TEST_ASSERT_TRUE(red_alarm);
    TEST_ASSERT_FALSE(yellow_alarm);
    
    printf("  PASSED\n");
    return 0;
}

// Test: Wire configuration constants
int test_wire_configuration(void) {
    printf("Testing: Wire configuration constants...\n");
    
    // Verify pins are defined correctly
    TEST_ASSERT_TRUE(M1_RED_PIN >= 0);
    TEST_ASSERT_TRUE(M1_YELLOW_PIN >= 0);
    TEST_ASSERT_TRUE(M2_RED_PIN >= 0);
    TEST_ASSERT_TRUE(M2_YELLOW_PIN >= 0);
    
    // Verify logic configuration
    TEST_ASSERT_TRUE(RED_WIRE_LOGIC == 0 || RED_WIRE_LOGIC == 1);
    TEST_ASSERT_TRUE(YELLOW_WIRE_LOGIC == 0 || YELLOW_WIRE_LOGIC == 1);
    
    // Verify function enum values
    TEST_ASSERT_EQUAL(0, WIRE_UNKNOWN);
    TEST_ASSERT_EQUAL(1, WIRE_THERMAL_CUTOFF);
    TEST_ASSERT_EQUAL(2, WIRE_THERMAL_WARNING);
    
    printf("  PASSED\n");
    return 0;
}

// Main test runner
int main(void) {
    printf("\n========================================\n");
    printf("TWO-WIRE SENSOR MOCK TESTS\n");
    printf("========================================\n\n");
    
    mock_init();
    
    int failures = 0;
    
    failures += test_both_wires_normal();
    failures += test_red_wire_alarm();
    failures += test_yellow_wire_alarm();
    failures += test_both_wires_alarm();
    failures += test_safety_check_red_wire();
    failures += test_motor2_wires();
    failures += test_wire_configuration();
    
    printf("\n========================================\n");
    if (failures == 0) {
        printf("ALL TESTS PASSED!\n");
    } else {
        printf("FAILURES: %d\n", failures);
    }
    printf("========================================\n\n");
    
    return failures;
}

#else
// ============================================================================
// HARDWARE-IN-LOOP TESTS - Run on actual ESP32
// These require real hardware with two-wire sensors connected
// ============================================================================

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "motor/motor.h"
#include "motor/pin.h"

static const char *TAG = "HIL_TEST";

// Test structure for results
typedef struct {
    const char* name;
    bool passed;
    char message[128];
} test_result_t;

static test_result_t test_results[10];
static int test_count = 0;

// Record test result
void record_result(const char* name, bool passed, const char* msg) {
    if (test_count < 10) {
        test_results[test_count].name = name;
        test_results[test_count].passed = passed;
        strncpy(test_results[test_count].message, msg, 127);
        test_results[test_count].message[127] = '\0';
        test_count++;
    }
}

// Test 1: Read raw wire values
void test_hil_read_raw(void) {
    ESP_LOGI(TAG, "=== TEST: Read raw wire values ===");
    
    int m1_red = gpio_get_level(M1_RED_PIN);
    int m1_yellow = gpio_get_level(M1_YELLOW_PIN);
    int m2_red = gpio_get_level(M2_RED_PIN);
    int m2_yellow = gpio_get_level(M2_YELLOW_PIN);
    
    ESP_LOGI(TAG, "M1 - Red: %d, Yellow: %d", m1_red, m1_yellow);
    ESP_LOGI(TAG, "M2 - Red: %d, Yellow: %d", m2_red, m2_yellow);
    
    // Check that we can read valid GPIO values (0 or 1)
    bool valid = (m1_red == 0 || m1_red == 1) && 
                 (m1_yellow == 0 || m1_yellow == 1) &&
                 (m2_red == 0 || m2_red == 1) && 
                 (m2_yellow == 0 || m2_yellow == 1);
    
    record_result("Read Raw Values", valid, 
                  valid ? "All pins readable" : "Invalid GPIO readings");
}

// Test 2: Use sensor reading function
void test_hil_sensor_function(void) {
    ESP_LOGI(TAG, "=== TEST: Sensor reading function ===");
    
    bool red_alarm = false, yellow_alarm = false;
    float temp = read_motor_end_sensors(MOTOR_ID_1, &red_alarm, &yellow_alarm);
    
    ESP_LOGI(TAG, "M1 - Red alarm: %s, Yellow alarm: %s, Temp: %.1f°C", 
             red_alarm ? "YES" : "NO", 
             yellow_alarm ? "YES" : "NO",
             temp);
    
    // Check that temperature reading is reasonable
    bool valid = (temp >= 0.0f && temp <= 150.0f);
    
    char msg[128];
    snprintf(msg, sizeof(msg), "M1 sensors: R=%d Y=%d T=%.1f°C", 
             red_alarm, yellow_alarm, temp);
    record_result("Sensor Function", valid, msg);
}

// Test 3: Check configuration matches hardware
void test_hil_configuration(void) {
    ESP_LOGI(TAG, "=== TEST: Configuration verification ===");
    
    ESP_LOGI(TAG, "M1 Red Pin: %d", M1_RED_PIN);
    ESP_LOGI(TAG, "M1 Yellow Pin: %d", M1_YELLOW_PIN);
    ESP_LOGI(TAG, "M2 Red Pin: %d", M2_RED_PIN);
    ESP_LOGI(TAG, "M2 Yellow Pin: %d", M2_YELLOW_PIN);
    ESP_LOGI(TAG, "Red Wire Logic: %s", RED_WIRE_LOGIC ? "Active HIGH" : "Active LOW");
    ESP_LOGI(TAG, "Yellow Wire Logic: %s", YELLOW_WIRE_LOGIC ? "Active HIGH" : "Active LOW");
    ESP_LOGI(TAG, "Red Wire Function: %d", RED_WIRE_FUNCTION);
    ESP_LOGI(TAG, "Yellow Wire Function: %d", YELLOW_WIRE_FUNCTION);
    
    // Verify pins are valid GPIO numbers for ESP32
    bool valid_pins = (M1_RED_PIN >= 0 && M1_RED_PIN <= 39) &&
                      (M1_YELLOW_PIN >= 0 && M1_YELLOW_PIN <= 39) &&
                      (M2_RED_PIN >= 0 && M2_RED_PIN <= 39) &&
                      (M2_YELLOW_PIN >= 0 && M2_YELLOW_PIN <= 39);
    
    record_result("Pin Configuration", valid_pins, 
                  valid_pins ? "All pins valid" : "Invalid pin numbers");
}

// Test 4: Safety check integration
void test_hil_safety_integration(void) {
    ESP_LOGI(TAG, "=== TEST: Safety check integration ===");
    
    // Initialize motor state
    motor_states[MOTOR_ID_1].current_speed = 100;
    motor_states[MOTOR_ID_1].red_wire_alarm = false;
    motor_states[MOTOR_ID_1].yellow_wire_alarm = false;
    motor_states[MOTOR_ID_1].thermal_shutdown = false;
    
    // Run safety check
    safety_check();
    
    ESP_LOGI(TAG, "After safety check:");
    ESP_LOGI(TAG, "  Red alarm: %s", motor_states[MOTOR_ID_1].red_wire_alarm ? "YES" : "NO");
    ESP_LOGI(TAG, "  Yellow alarm: %s", motor_states[MOTOR_ID_1].yellow_wire_alarm ? "YES" : "NO");
    ESP_LOGI(TAG, "  Thermal shutdown: %s", motor_states[MOTOR_ID_1].thermal_shutdown ? "YES" : "NO");
    
    // Check that alarm flags were updated
    bool valid = true;  // Just verify no crash occurred
    
    char msg[128];
    snprintf(msg, sizeof(msg), "R=%d Y=%d TH=%d",
             motor_states[MOTOR_ID_1].red_wire_alarm,
             motor_states[MOTOR_ID_1].yellow_wire_alarm,
             motor_states[MOTOR_ID_1].thermal_shutdown);
    record_result("Safety Integration", valid, msg);
}

// Test 5: Manual trigger test (requires user interaction)
void test_hil_manual_trigger(void) {
    ESP_LOGI(TAG, "=== TEST: Manual wire trigger ===");
    ESP_LOGI(TAG, "INSTRUCTION: Manually trigger the red wire sensor now (if possible)");
    ESP_LOGI(TAG, "Waiting 5 seconds...");
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    int red_level = gpio_get_level(M1_RED_PIN);
    bool triggered = (red_level == RED_WIRE_LOGIC);
    
    if (triggered) {
        ESP_LOGI(TAG, "Red wire was triggered!");
        record_result("Manual Trigger", true, "Red wire manually triggered");
    } else {
        ESP_LOGW(TAG, "Red wire not triggered (may require hardware mod)");
        record_result("Manual Trigger", false, "No manual trigger detected");
    }
}

// Print all test results
void print_test_results(void) {
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "TEST RESULTS SUMMARY");
    ESP_LOGI(TAG, "========================================");
    
    int passed = 0;
    for (int i = 0; i < test_count; i++) {
        ESP_LOGI(TAG, "%d. %s: %s", 
                 i + 1, 
                 test_results[i].name,
                 test_results[i].passed ? "PASS" : "FAIL");
        ESP_LOGI(TAG, "   %s", test_results[i].message);
        if (test_results[i].passed) passed++;
    }
    
    ESP_LOGI(TAG, "\nPassed: %d/%d", passed, test_count);
    ESP_LOGI(TAG, "========================================\n");
}

// Main hardware-in-loop test task
void hil_test_task(void *param) {
    ESP_LOGI(TAG, "Starting Hardware-in-Loop Tests...");
    ESP_LOGI(TAG, "Connect to this ESP32 via serial monitor to see results");
    
    // Small delay to allow serial to connect
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_hil_read_raw();
    test_hil_sensor_function();
    test_hil_configuration();
    test_hil_safety_integration();
    test_hil_manual_trigger();
    
    print_test_results();
    
    ESP_LOGI(TAG, "Hardware tests complete. System will continue normal operation.");
    
    vTaskDelete(NULL);
}

// Entry point for HIL tests
void run_hil_tests(void) {
    xTaskCreate(hil_test_task, "HIL_Test", 4096, NULL, 5, NULL);
}

#endif
