#pragma once
#include <stdint.h>

void proportional(uint16_t& plant, float val, float desired, float k, float step); // Proportional Control in PID
void integral(uint16_t& plant, float* recentVals, float desired, float k, float step); // Integral Control in PID
void derivative(uint16_t& plant, float curVal, float prevVal, float k, float step); // Derivative Control in PID