#pragma once
#include <stdint.h>

struct msgData{
    int8_t x_change; // Forwards/backwards
    int8_t y_change; // Sideways
    int8_t z_change; // Height
};

void remote_control_init(msgData *msg); // initialize new msgData

bool isStable(msgData msg); // Returns true if all change vals are 0
bool xStable(msgData msg); // Returns true if y_change is 0
bool yStable(msgData msg); // Returns true if x_change is 0