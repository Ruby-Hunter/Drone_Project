#pragma once

#include <stdint.h>
#include <stddef.h>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline void delay(unsigned long) {}

template <typename T>
inline T constrain(T val, T minVal, T maxVal) {
    if(val < minVal) return minVal;
    if(val > maxVal) return maxVal;
    return val;
}

template <typename T1, typename T2, typename T3, typename T4, typename T5>
inline auto map(T1 x, T2 in_min, T3 in_max, T4 out_min, T5 out_max) -> decltype(x + out_min) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

