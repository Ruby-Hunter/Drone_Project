#pragma once
#include <stdint.h>

void proportional(uint16_t& plant, float val, float desired, float k, float step); // Proportional Control in PID
void integral(uint16_t& plant, float* recentVals, float desired, float k, float step); // Integral Control in PID
void integral(uint16_t& plant, const int16_t* recentVals, float desired, float k, float step);
void derivative(uint16_t& plant, float curVal, float prevVal, float desired, float k, float step); // Derivative Control in PID
double correctAltitude(int16_t lidar_distance_mm, float accel_x_g, float accel_y_g); // Corrects lidar distance based on tilt of drone