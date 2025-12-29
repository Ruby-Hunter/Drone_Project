// #include "../../../git/Unity/src/unity.h" // Blake: comment, when testing, uncomment when writing code
#include <unity.h>
#include "motor_control.h"
#include "test_motor_control.h"
#include "test_sensors.h"

/* ----- We need these 2 functions to exist for it to run ----- */
void setUp(void) {
    // This is run before EACH TEST
}
void tearDown(void) {
    // This is run after EACH TEST
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_motor_init);
    RUN_TEST(test_setSpeed);
    RUN_TEST(test_changeSpeed);
    RUN_TEST(test_over_max_speed);
    RUN_TEST(test_under_max_speed);
    RUN_TEST(test_stopMotors);
    RUN_TEST(test_balancePitch);
    RUN_TEST(test_balanceRoll);
    RUN_TEST(test_readGyro);
    return UNITY_END();
}