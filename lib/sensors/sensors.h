#pragma once
#include <stdint.h>
#include <Adafruit_DPS310.h>
#include <Arduino.h>

/* ----- Sensor Data Structure ----- */
struct SensorData { // Stores the last few readings from each sensor
    float accel_x_g = 0;
    float accel_y_g = 0;
    float accel_z_g = 0;
    float gyro_x_dps = 0;
    float gyro_y_dps = 0;
    float gyro_z_dps = 0;
    int16_t lidar_distance_mm = 0;
    sensors_event_t pressure = {};
};

struct SensorDataHistory { // Stores the last few readings from each sensor
    uint8_t index = 0;
    uint8_t prev_index = 0;
    float accel_x_g[NUM_DATA_VALS] = {0};
    float accel_y_g[NUM_DATA_VALS] = {0};
    float accel_z_g[NUM_DATA_VALS] = {0};
    float gyro_x_dps[NUM_DATA_VALS] = {0};
    float gyro_y_dps[NUM_DATA_VALS] = {0};
    float gyro_z_dps[NUM_DATA_VALS] = {0};
    int16_t lidar_distance_mm[NUM_DATA_VALS] = {0};
    sensors_event_t pressure[NUM_DATA_VALS] = {};
};

/* ----- Sensor Setup ----- */
void setupSensors();
void initDrivers(); // Sets sensor function pointers to HW versions

/* ----- Sensor Reading Functions ----- */
void readSensors(SensorData& data); // Read all sensors
void advanceIndex(SensorDataHistory& history); // Advances index, called before reading new sensor data
void updateHistory(SensorDataHistory& history, SensorData& data); // Updates history with new sensor data

void readGyro(SensorData& data); // Read from Gyro
void readLidar(SensorData& data); // Read from Lidar TOF sensor
void readPressure(SensorData& data); // Read from pressure sensor

/* ----- Sensor function selection for testing ----- */
using SensorFunc = void(*)(SensorData& data);
void setGyroFunc(SensorFunc);
void setLidarFunc(SensorFunc);
void setPressureFunc(SensorFunc);