#pragma once

class TwoWire {
public:
    void begin() {}
    void setSCL(int pin) {}
    void setSDA(int pin) {}
};

extern TwoWire Wire;
