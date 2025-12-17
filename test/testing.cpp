#include "../../../git/Unity/src/unity.h"
#include <string.h>
#include <stdlib.h>
#include "main.cpp"

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