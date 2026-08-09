#pragma once
#include <stdint.h>

#ifdef UNIT_TEST
  #define MOTOR1 1
  #define MOTOR2 2
  #define MOTOR3 3
  #define MOTOR4 4
  #define LED_PIN 13
  #define DRONE_POWER 5
  #define I2C_SCL 6
  #define I2C_SDA 7
#else
  #define MOTOR1 PA0
  #define MOTOR2 PA8
  #define MOTOR3 PA9
  #define MOTOR4 PA10
  #define LED_PIN PC13
  #define DRONE_POWER PA4
  #define I2C_SCL PB6
  #define I2C_SDA PB7
#endif


const uint32_t DELAY_MS = 50;
const uint32_t STARTUP_TIME_MS = 1000;

// Conversion factors
const float TWO_GS_FORCE = 16384.0f;
const float GYRO_DPS = 131.0f; // Divide raw data by this for Degrees Per Second (DPS)
const float PRESSURE_CHANGE_TO_ALTITUDE_MM = 0.0001f; // 1 hPa = 0.01 m altitude

// Thresholds
const float ALTITUDE_THRESHOLD_MM = 25.0f; // in mm
const float MOTION_THRESHOLD = 0.1f;  // in G's
const float PRESSURE_THRESHOLD_HPA = 0.01f; // In hPa
const uint8_t MOVEMENT_THRESHOLD = 10;

// PID Constants
const long MAX_PWM_CHANGE = 6;
const uint8_t NUM_DATA_VALS = 5; // How many past sensor vals stored for I term

const float DESIRED_ROLL = 0.0f;
const float DESIRED_PITCH = 0.0f;
const float DESIRED_YAW = 0.0f;

const float P_PITCH = 5;
const float I_PITCH = 3;
const float D_PITCH = 2;

const float P_ROLL = 5;
const float I_ROLL = 3;
const float D_ROLL = 2;

const float P_ALTITUDE = 5;
const float I_ALTITUDE = 3;
const float D_ALTITUDE = 2;

const float P_YAW = 5;
const float I_YAW = 3;
const float D_YAW = 2;

// Sensor Constants
const float ALPHA = 0.5;
const float ACCEL_LP_MAX_G = 0.5; // LP = Low Pass
const float GYRO_LP_MAX_DPS = 30; // LP = Low Pass

// Take off Constants
const long TAKEOFF_SPEED = 8;

// Motor speed constants (in microseconds)
const uint16_t STOP_SPEED = 1000;
const uint16_t START_SPEED = 1150;
const uint16_t RISING_HOVER_SPEED = 1360;//unused
const uint16_t MAX_HOVER_SPEED = 1200;//unused
const uint16_t MAX_SPEED = 1550;

// Other constants
const int16_t TAKEOFF_HEIGHT_MM = 500; // Target height for takeoff in mm
const int16_t LANDING_DISTANCE_MM = 60; // Distance from sensor that triggers landing sequence

// When the drone goes above this height, initiate landing sequence
const float HEIGHT_LIMIT_HPA = TAKEOFF_HEIGHT_MM*3*PRESSURE_CHANGE_TO_ALTITUDE_MM;