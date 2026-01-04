#pragma once
#include <stdint.h>

class MPU6050 {
public:
    void initialize() {}
    void setFullScaleGyroRange(int range) {}
    void setFullScaleAccelRange(int range) {}
    void getAcceleration(int16_t* ax, int16_t* ay, int16_t* az) {
        *ax = *ay = *az = 0;
    }
    void getRotation(int16_t* gx, int16_t* gy, int16_t* gz) {
        *gx = *gy = *gz = 0;
    }
};

#define MPU6050_GYRO_FS_250 0
#define MPU6050_ACCEL_FS_2 0
