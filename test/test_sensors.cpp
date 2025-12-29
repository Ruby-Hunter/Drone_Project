// #include "../../../git/Unity/src/unity.h" // Blake: comment, when testing, uncomment when writing code
#include <unity.h>
#include <stdlib.h>
#include "sensors.h"
#include "config.h"

/* ----- Dummy Sensor Functions for Testing ----- */
void dummyGyro(SensorData& data){
    data.accel_x_g = 0.0f;
    data.accel_y_g = 0.0f;
    data.accel_z_g = 1.0f;
    data.gyro_x_dps = 0.0f;
    data.gyro_y_dps = 0.0f;
    data.gyro_z_dps = 0.0f;
}

/* ----- TESTS ----- */
void test_readGyro(){
    setGyroFunc(dummyGyro);
    SensorData data;
    readGyro(data);
    // Check that values are within reasonable ranges
    TEST_ASSERT(data.accel_x_g < 2.0f && data.accel_x_g > -2.0f);
    TEST_ASSERT(data.accel_y_g < 2.0f && data.accel_y_g > -2.0f);
    TEST_ASSERT(data.accel_z_g < 2.0f && data.accel_z_g > -2.0f);
    TEST_ASSERT(data.gyro_x_dps < 500.0f && data.gyro_x_dps > -500.0f);
    TEST_ASSERT(data.gyro_y_dps < 500.0f && data.gyro_y_dps > -500.0f);
    TEST_ASSERT(data.gyro_z_dps < 500.0f && data.gyro_z_dps > -500.0f);
}

void test_readLidar(){
    setLidarFunc([](SensorData& data){
        data.distance_mm = 1000; // Dummy distance
    });
    SensorData data;
    readLidar(data);
    TEST_ASSERT_EQUAL_INT(1000, data.distance_mm);
}