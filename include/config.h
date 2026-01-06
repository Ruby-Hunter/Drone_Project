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
  #define MOTOR1 PA8
  #define MOTOR2 PA9
  #define MOTOR3 PA10
  #define MOTOR4 PA0
  #define LED_PIN PC13
  #define DRONE_POWER PB4
  #define I2C_SCL PB6
  #define I2C_SDA PB7
#endif


const uint32_t MS_DELAY = 50;

const float PRESSURE_CHANGE_TO_ALTITUDE_MM = 0.0001f; // 1 hPa = 0.01 m altitude
const float ALTITUDE_THRESHOLD_MM = 50.0f; // in mm
const float MOTION_THRESHOLD = 0.1f;  // in G's
const float PRESSURE_THRESHOLD = 0.01f; // In hPa
const uint8_t MOVEMENT_THRESHOLD = 10;
const float TWO_GS_FORCE = 16384.0f;
const float GYRO_DPS = 131.0f; // Divide raw data by this for Degrees Per Second (DPS)

// PID Constants
const long MAX_CHANGE = 8;

// Take off Constants
const long TAKEOFF_SPEED = 8;

// Motor speed constants (in microseconds)
const uint16_t STOP_SPEED = 1000;
const uint16_t START_SPEED = 1320;
const uint16_t RISING_HOVER_SPEED = 1360;
const uint16_t MAX_HOVER_SPEED = 1550;
const uint16_t MAX_SPEED = 1550;

// Other constants
const int16_t TAKEOFF_HEIGHT_MM = 1000; // Target height for takeoff in mm
const int16_t LANDING_DISTANCE_MM = 200; // Distance from sensor that triggers landing sequence