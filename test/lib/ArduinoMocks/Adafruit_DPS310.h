#pragma once
#include <stdint.h>
#include "Wire.h"

struct sensors_event_t {
    float pressure = 1000.0f;
};

class Adafruit_DPS310 {
public:
    bool begin_I2C(uint8_t addr, TwoWire* wire = nullptr) { return true; }
    void configurePressure(int, int) {}
    void configureTemperature(int, int) {}
    void getEvents(sensors_event_t* temp, sensors_event_t* pressure) {
        pressure->pressure = 1000.0f;
    }
};

#define DPS310_64HZ 0
#define DPS310_16SAMPLES 0
