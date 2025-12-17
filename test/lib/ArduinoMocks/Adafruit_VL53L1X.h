#pragma once
#include <stdint.h>

class Adafruit_VL53L1X {
public:
    bool begin(uint8_t addr, void* wire = nullptr) { return true; }
    void startRanging() {}
    bool dataReady() { return false; }
    int distance() { return 1000; }
    void clearInterrupt() {}
};
