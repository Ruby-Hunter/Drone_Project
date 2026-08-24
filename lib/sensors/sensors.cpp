#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_VL53L1X.h>
#include <Arduino.h>

// sensor objects
Adafruit_VL53L1X lidar;
Adafruit_DPS310 dps;
MPU6050 mpu;

// values for readings
static int16_t ax, ay, az;
static int16_t gx, gy, gz;
static sensors_event_t temp, pressure;

// Active function pointers
static SensorFunc gyroFunc;
static SensorFunc lidarFunc;
static SensorFunc pressureFunc;

template <typename T>
static T alphaFilter(T lastVal, T curVal){
    return (ALPHA * curVal) + ((1-ALPHA) * lastVal);
}

template <typename T>
static bool lowPassFilter(T lastVal, T curVal, T max){
    return abs(curVal - lastVal) < max;
}

/* ----- Hardware reading functions ----- */
static void readGyroHW(SensorData& data){
    // x and y are switched because I put on the gyro sensor sideways
    mpu.getAcceleration(&ay, &ax, &az);
    mpu.getRotation(&gy, &gx, &gz);

    // Accelerometer Range: [-2, 2]. If it passes low pass filter, apply alpha filter and update data
    float ax_g = ax / TWO_GS_FORCE;
    float ay_g = ay / TWO_GS_FORCE;
    float az_g = az / TWO_GS_FORCE;
    // if(lowPassFilter(data.accel_x_g, ax_g, ACCEL_LP_MAX_G)){ data.accel_x_g = alphaFilter(data.accel_x_g, ax_g); }
    // if(lowPassFilter(data.accel_y_g, ay_g, ACCEL_LP_MAX_G)){ data.accel_y_g = alphaFilter(data.accel_y_g, ay_g); }
    // if(lowPassFilter(data.accel_z_g, az_g, ACCEL_LP_MAX_G)){ data.accel_z_g = alphaFilter(data.accel_z_g, az_g); }

    // Gyro Range: [-180, 180]
    float gx_dps = gx / GYRO_DPS;
    float gy_dps = gy / GYRO_DPS;
    float gz_dps = gz / GYRO_DPS;
    // if(lowPassFilter(data.gyro_x_dps, gx_dps, GYRO_LP_MAX_DPS)){ data.gyro_x_dps = alphaFilter(data.gyro_x_dps, gx_dps); }
    // if(lowPassFilter(data.gyro_y_dps, gy_dps, GYRO_LP_MAX_DPS)){ data.gyro_y_dps = alphaFilter(data.gyro_y_dps, gy_dps); }
    // if(lowPassFilter(data.gyro_z_dps, gz_dps, GYRO_LP_MAX_DPS)){ data.gyro_z_dps = alphaFilter(data.gyro_z_dps, gz_dps); }

    data.accel_x_g = ax_g;
    data.accel_y_g = ay_g;
    data.accel_z_g = az_g;
    data.gyro_x_dps = gx_dps;
    data.gyro_y_dps = gy_dps;
    data.gyro_z_dps = gz_dps;
}

static void readLidarHW(SensorData& data){
  if (lidar.dataReady()) {
    if(lidar.distance() != -1){
        // data.lidar_distance_mm = alphaFilter(data.lidar_distance_mm, lidar.distance()); // Distance in millimeters
        data.lidar_distance_mm = lidar.distance();
    }
    lidar.clearInterrupt(); // Reset data ready flag
  }
}

static void readPressureHW(SensorData& data){ //TODO: filter
  dps.getEvents(&temp, &pressure);
  data.pressure = pressure;
}


/* ----- Sensor Setup and Reading Functions ----- */
void setupSensors(){
    Serial.println("Initializing I2C...");
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin(); // default SCL = PB6, SDA = PB7
  Wire.setClock(400000); // 400kHz I2C speed

    Serial.println("Initializing Gyro...");
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_98); // 20, 42, 98, 188, 256 Hz
  mpu.setRate(0); // 1000 Hz sample rate (1000 / (1 + [0]) = 1000)

    Serial.println("Initializing LIDAR...");
  bool lidarStart = lidar.begin(0x29, &Wire);
  lidar.startRanging();

    Serial.println("Initializing Pressure...");
  bool dpsStart = dps.begin_I2C(0x77, &Wire);
  dps.configurePressure(DPS310_64HZ, DPS310_8SAMPLES);
  dps.configureTemperature(DPS310_16HZ, DPS310_4SAMPLES);

    Serial.println("Setting up sensor function pointers...");
  initDrivers();
  Serial.println("Sensors initialized");
}

void initDrivers(){
    gyroFunc = readGyroHW;
    lidarFunc = readLidarHW;
    pressureFunc = readPressureHW;
}

void readGyro(SensorData& data){
    if(gyroFunc != nullptr){
        gyroFunc(data);
    }
}

void readLidar(SensorData& data){
    if(lidarFunc != nullptr){
        lidarFunc(data);
    }
}

void readPressure(SensorData& data){
    if(pressureFunc != nullptr){
        pressureFunc(data);
    }
}

void readSensors(SensorData& data){
    // readGyro(data);
    readLidar(data);
    readPressure(data);
}

void advanceIndex(SensorDataHistory& history){
    history.prev_index = history.index;
    history.index = (history.index + 1) % NUM_DATA_VALS;
    // history.prev_imu_idx = history.imu_idx;
    // history.imu_idx = (history.imu_idx + 1) % NUM_DATA_VALS;
}

void advanceIMUIndex(SensorDataHistory& history){
    history.prev_imu_idx = history.imu_idx;
    history.imu_idx = (history.imu_idx + 1) % NUM_DATA_VALS;
}

void updateHistory(SensorDataHistory& history, SensorData& data){
    uint8_t idx = history.index;
    uint8_t imu_idx = history.imu_idx;
    history.accel_x_g[imu_idx] = data.accel_x_g;
    history.accel_y_g[imu_idx] = data.accel_y_g;
    history.accel_z_g[imu_idx] = data.accel_z_g;
    history.gyro_x_dps[imu_idx] = data.gyro_x_dps;
    history.gyro_y_dps[imu_idx] = data.gyro_y_dps;
    history.gyro_z_dps[imu_idx] = data.gyro_z_dps;
    history.lidar_distance_mm[idx] = data.lidar_distance_mm;
    history.pressure[idx] = data.pressure;
}

bool readingsStable(SensorData data){
  return abs(data.accel_x_g) < MOTION_THRESHOLD && abs(data.accel_y_g) < MOTION_THRESHOLD;
    // return abs(data.accel_x_g) < MOTION_THRESHOLD && abs(data.accel_y_g) < MOTION_THRESHOLD && abs(data.accel_z_g) < MOTION_THRESHOLD;

}

void setGyroFunc(SensorFunc func) {
    gyroFunc = func;
}
void setLidarFunc(SensorFunc func) {
    lidarFunc = func;
}
void setPressureFunc(SensorFunc func) {
    pressureFunc = func;
}