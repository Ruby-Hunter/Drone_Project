#pragma once
#include <stdint.h>
#include <variant>

using Num = std::variant<int, float, double, uint16_t, uint32_t>;

void proportional(Num& plant, Num val, Num desired, Num k, Num step); // Proportional Control in PID
void integral(Num& plant, Num* recentVals, Num desired, Num k, Num step); // Integral Control in PID
void derivative(Num& plant, Num curVal, Num prevVal, Num k, Num step); // Derivative Control in PID