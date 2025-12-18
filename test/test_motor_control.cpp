// #include "../../../git/Unity/src/unity.h" // Blake: comment, when testing, uncomment when writing code
#include <unity.h>
#include <stdlib.h>
#include "motor_control.h"
#include "config.h"

static void assert_all_motors(uint16_t speed, bool equal = true){ // Helper function for tests
    for(int i = 0; i < 4; i++){
        if(equal){
            TEST_ASSERT_EQUAL(speed, getSpeed(i));
        } else{
            TEST_ASSERT_NOT_EQUAL(speed, getSpeed(i));
        }
        
    }
}

/* ----- We need these 2 functions to exist for it to run ----- */
void setUp(void) {
    // This is run before EACH TEST
}
void tearDown(void) {
    // This is run after EACH TEST
}

/* ----- TESTS ----- */
void test_motor_init(){
    motor_init();
    assert_all_motors(STOP_SPEED);
}

void test_setSpeed(){
    uint16_t newSpeed = 1199;
    setSpeed(newSpeed);
    assert_all_motors(newSpeed);
}

void test_changeSpeed(){
    uint16_t starterSpeed = 1099;
    setSpeed(starterSpeed);
    assert_all_motors(starterSpeed);

    changeSpeed(1);
    assert_all_motors(starterSpeed + 1);

    changeSpeed(-3);
    assert_all_motors(starterSpeed - 2);
}

void test_over_max_speed(){
    uint16_t overMaxSpeed = 2050;

    // setSpeed should set it to overmax
    setSpeed(overMaxSpeed);
    assert_all_motors(overMaxSpeed); 

    // writeESCs should cap the value at MAX_SPEED
    writeESCs();
    assert_all_motors(overMaxSpeed, false); 
    assert_all_motors(MAX_SPEED); 
}

void test_under_max_speed(){
    uint16_t underMaxSpeed = 900;

    // setSpeed should set it to overmax
    setSpeed(underMaxSpeed);
    assert_all_motors(underMaxSpeed); 

    // writeESCs should cap the value at STOP_SPEED
    writeESCs();
    assert_all_motors(underMaxSpeed, false); 
    assert_all_motors(STOP_SPEED); 
}

void test_stopMotors(){
    uint16_t speed = 1300;

    motor_init();
    setSpeed(speed);
    assert_all_motors(speed);

    stopMotors();
    assert_all_motors(STOP_SPEED);
}

void test_balancePitch(){
    float accel_x_g[] = {1, 0, -0.5};

    motor_init();
    setSpeed(START_SPEED);

    balancePitch(accel_x_g[0], 0);
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(1));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(2));

    balancePitch(accel_x_g[1], 0);
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(1));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(2));
    TEST_ASSERT_EQUAL(START_SPEED, getSpeed(3));
    TEST_ASSERT_EQUAL(START_SPEED, getSpeed(4));

    balancePitch(accel_x_g[2], 0);
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(1));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(2));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE/2 + 1, getSpeed(3));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE/2 + 1, getSpeed(4));
}

void test_balanceRoll(){
    float accel_y_g[] = {1, 0, -0.5};

    motor_init();
    setSpeed(START_SPEED);

    balanceRoll(accel_y_g[0], 0);
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(2));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(3));

    balanceRoll(accel_y_g[1], 0);
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(2));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(3));
    TEST_ASSERT_EQUAL(START_SPEED, getSpeed(1));
    TEST_ASSERT_EQUAL(START_SPEED, getSpeed(4));

    balanceRoll(accel_y_g[2], 0);
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 2, getSpeed(2));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE + 1, getSpeed(3));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE/2 + 1, getSpeed(1));
    TEST_ASSERT_EQUAL(START_SPEED + MAX_CHANGE/2 + 1, getSpeed(4));
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
    return UNITY_END();
}