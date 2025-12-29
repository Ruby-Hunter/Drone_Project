#include "sensors.h"
#include "config.h"

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

/* ----- Hardware reading functions ----- */
static void readGyroHW(SensorData& data){
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    data.accel_x_g = ax / TWO_GS_FORCE;
    data.accel_y_g = ay / TWO_GS_FORCE;
    data.accel_z_g = az / TWO_GS_FORCE;
    data.gyro_x_dps = gx / GYRO_DPS;
    data.gyro_y_dps = gy / GYRO_DPS;
    data.gyro_z_dps = gz / GYRO_DPS;
}

static void readLidarHW(SensorData& data){
  if (lidar.dataReady()) {
    data.distance_mm = lidar.distance(); // Distance in millimeters
    lidar.clearInterrupt(); // Reset data ready flag
  }
}

static void readPressureHW(SensorData& data){
  dps.getEvents(&temp, &pressure);
  data.pressure = pressure;
}

/* ----- Sensor Setup and Reading Functions ----- */
void setupSensors(){
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin(); // default SCL = PB6, SDA = PB7

  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  bool lidarStart = lidar.begin(0x29, &Wire);
  lidar.startRanging();

  bool dpsStart = dps.begin_I2C(0x77, &Wire);
  dps.configurePressure(DPS310_64HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_16SAMPLES);

  initDrivers();
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

void readValues(SensorData& data){
    readGyro(data);
    readLidar(data);
    readPressure(data);
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