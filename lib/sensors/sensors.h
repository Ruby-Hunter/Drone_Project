#pragma once
#include <stdint.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_DPS310.h>

/* ----- Sensor Data Structure ----- */
struct SensorData {
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    int16_t distance_mm;
    sensors_event_t pressure;
};

/* ----- Sensor Setup ----- */
void setupSensors();
void initDrivers(); // Sets sensor function pointers to HW versions

/* ----- Sensor Reading Functions ----- */
void readValues(SensorData& data); // Read all sensors

void readGyro(SensorData& data); // Read from Gyro
void readLidar(SensorData& data); // Read from Lidar TOF sensor
void readPressure(SensorData& data); // Read from pressure sensor

/* ----- Sensor function selection for testing ----- */
using SensorFunc = void(*)(SensorData& data);
void setGyroFunc(SensorFunc);
void setLidarFunc(SensorFunc);
void setPressureFunc(SensorFunc);