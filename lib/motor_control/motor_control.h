#pragma once
#include <stdint.h>

void motor_init(); // Set up motors

/* ----- FLIGHT CONTROL FUNCTIONS ----- */
void setSpeed(uint16_t newSpeed); // Sets speed of all motors to newSpeed
void changeSpeed(int16_t change); // Adjusts all motor speeds by change
void moveX(int8_t x_movement); // Moves the drone to x_movement pos forwards/backwards
void moveY(int8_t y_movement); // Moves teh drone to y_movement pos right/left
void moveZ(int8_t z_movement); // Moves the drone up/down z_movement distance
void writeESCs(); // Writes new PWM values to motors
void stopMotors(); // Sets all motors to 1000

void balancePitch(float accel_x_g, float gyro_x_dps); // Balances motors based on pitch
void balanceRoll(float accel_y_g, float gyro_y_dps); // Balances motors based on roll
void balanceAltitude(float pressure, float hoverPressure); // Checks for changes in pressure, adjusts all motors based on it
void balanceAltitudeLidar(float height_mm); // Balances altitude based off of lidar sensor

void takeOff(int16_t distance_mm); // Takeoff sequence

void printSpeeds(); // Prints out current motor speeds

//TODO: Move land and forceLand over here from main
// void land(); // Landing sequence
// void forceLand(); // Force the drone to land if pressure sensor isn't working

/* ----- TESTING FUNCTIONS ----- */
uint16_t getSpeed(int motorChoice = 4); // Gets speed of motorChoice motor; default is 4
bool motorSpeedsEqual(uint16_t speed); // Checks if all motor speeds are equal to 'speed' value