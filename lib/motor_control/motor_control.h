#pragma once
#include <stdint.h>

void motor_init(); // Set up motors

/* ----- FLIGHT CONTROL FUNCTIONS ----- */
void setSpeed(uint16_t newSpeed); // Sets speed of all motors to newSpeed
void changeSpeed(int16_t change); // Adjusts all motor speeds by change
void writeESCs(); // Writes new PWM values to motors
void stopMotors(); // Sets all motors to 1000

void balancePitch(float accel_x_g); // Balances motors based on pitch
void balanceRoll(float accel_y_g); // Balances motors based on roll
void balanceAltitude(float pressure, float hoverPressure); // Checks for changes in verticle position, adjusts all motors based on it

void takeOff(); // Takeoff sequence
void land(); // Landing sequence
void forceLand(); // force the drone to land if pressure sensor isn't working

/* ----- TESTING FUNCTIONS ----- */
uint16_t getSpeed(int motorChoice = 4); // Gets speed of motorChoice motor; default is 4