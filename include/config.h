#pragma once
#include <stdint.h>

#ifdef UNIT_TEST
  #define MOTOR1 1
  #define MOTOR2 2
  #define MOTOR3 3
  #define MOTOR4 4
  #define LED_PIN 13
  #define DRONE_POWER 5
#else
  #define MOTOR1 PA8
  #define MOTOR2 PA9
  #define MOTOR3 PA10
  #define MOTOR4 PA11
  #define LED_PIN PC13
  #define DRONE_POWER PB4
#endif


const uint32_t MS_DELAY = 50;

const float MOTION_THRESHOLD = 0.1f;  // in G's
const float PRESSURE_THRESHOLD = 5.0f; // In hPa
const uint8_t MOVEMENT_THRESHOLD = 10;
const float TWO_GS_FORCE = 16384.0f;
const float GYRO_DPS = 131.0f; // Divide raw data by this for Degrees Per Second (DPS)

// PID Constants
const long MAX_CHANGE = 8;

// Motor speed constants (in microseconds)
const uint16_t STOP_SPEED = 1000;
const uint16_t START_SPEED = 1320;
const uint16_t RISING_HOVER_SPEED = 1360;
const uint16_t MAX_HOVER_SPEED = 1550;
const uint16_t MAX_SPEED = 1550;

// Other constants
const int16_t LANDING_DISTANCE_MM = 200;