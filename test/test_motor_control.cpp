#include <unity.h>
#include <stdlib.h>
#include "motor_control.h"

void setUp(void) {
    // This is run before EACH TEST
}
void tearDown(void) {
    // This is run after EACH TEST
}

void test_setSpeed(){
    uint16_t newSpeed = 1199;
    setSpeed(newSpeed);
    TEST_ASSERT_EQUAL(newSpeed, getSpeed(1));
    TEST_ASSERT_EQUAL(newSpeed, getSpeed(2));
    TEST_ASSERT_EQUAL(newSpeed, getSpeed(3));
    TEST_ASSERT_EQUAL(newSpeed, getSpeed(4));
}

void test_changeSpeed(){
    uint16_t starterSpeed = 1099;
    setSpeed(starterSpeed);
    TEST_ASSERT_EQUAL(starterSpeed, getSpeed(1));

    changeSpeed(1);
    TEST_ASSERT_EQUAL(starterSpeed + 1, getSpeed(1));

    changeSpeed(-3);
    TEST_ASSERT_EQUAL(starterSpeed - 2, getSpeed(1));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_setSpeed);
    RUN_TEST(test_changeSpeed);
    return UNITY_END();
}