#pragma once
#include <stdint.h>
#include <Adafruit_DPS310.h>
#include <Arduino.h>

/* ----- Sensor Data Structure ----- */
struct SensorData {
    float accel_x_g = 0;
    float accel_y_g = 0;
    float accel_z_g = 1;
    float gyro_x_dps = 0;
    float gyro_y_dps = 0;
    float gyro_z_dps = 0;
    int16_t distance_mm = 90;
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