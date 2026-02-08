#include <unity.h>
#include <cmock.h>
#include <motor/motor.h>

static int mock_gpio_level = 0;

void test_limit_switch_top_m1_pressed() {
    mock_gpio_level = 0;  // Active low
    bool result = check_limit_switch(MOTOR_ID_1, true);
    TEST_ASSERT_EQUAL(true, result);
}

void app_main()
{
  UNITY_BEGIN();

  RUN_TEST(test_limit_switch_top_m1_pressed);

  UNITY_END();
}