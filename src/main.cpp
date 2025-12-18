#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_DPS310.h>
#include <cmath>
#include "motor_control.h"
#include "remote_control.h"
#include "config.h"

Servo sx, sy, sz, sdist, spres; // Saleae testing

// Initialize sensor objects
Adafruit_VL53L1X lidar = Adafruit_VL53L1X();
Adafruit_DPS310 dps;
MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float accel_x_g, accel_y_g, accel_z_g;
float gyro_x_dps, gyro_y_dps, gyro_z_dps;
int16_t distance; // Distance in mm
sensors_event_t temp, pressure;

float basePressure, hoverPressure;

// For Testing
int16_t pwmx, pwmy, pwmz, pwmdist, pwmpres;

enum State { OFF, INIT, HOVERING, FLYING, LANDING, TESTING };
uint8_t currentState = TESTING;

msgData msg;

void setup() {
  setupPins();
  setupServos();
  setupSensors();
  setupRC();
  tripleBlink();
  delay(1000);
}

void loop() {
  stateMachine();
  delay(MS_DELAY);
}

void stateMachine(){
  if(!digitalRead(DRONE_POWER)){ // Failsafe
    currentState = OFF;
    stopMotors();
  }

  switch (currentState){
    case OFF: 
      if(digitalRead(DRONE_POWER)){
        currentState = INIT;
      }
      break;

    case INIT:
      currentState = HOVERING;
      break;

    case HOVERING:
      if(distance < LANDING_DISTANCE_MM){
        currentState = LANDING;
      } else if(!isStable(msg)){
        currentState = FLYING;
      }
      break;
    
    case FLYING:
      if(distance < LANDING_DISTANCE_MM){
        currentState = LANDING;
      } else if(isStable(msg)){
        currentState = HOVERING;
      }
      break;
    
    case LANDING:
      currentState = TESTING;
      break;
  }

  switch (currentState){
    case OFF:
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      break;
    
    case INIT:
      tripleBlink();
      delay(3000);  // Wait for ESCs to initialize
      readPressure();
      basePressure = pressure.pressure; // pressure.altitude?
      takeOff();
      readPressure();
      hoverPressure = pressure.pressure;
      break;
    
    case HOVERING:
      readValues();
      sendReadings();

      balancePitch(accel_x_g, gyro_x_dps);
      balanceRoll(accel_y_g, gyro_y_dps);
      balanceAltitude(pressure.pressure, hoverPressure);

      writeESCs();
      break;
    
    case FLYING:
      readValues();
      sendReadings();

      if(xStable(msg)) balancePitch(accel_x_g, gyro_x_dps); // Keep balanced if no move command, else move
      else moveX(msg.x_change);
      if(yStable(msg)) balanceRoll(accel_y_g, gyro_y_dps);
      else moveY(msg.y_change);
      balanceAltitude(pressure.pressure, hoverPressure);

      writeESCs();
      break;

    case LANDING:
      digitalWrite(LED_PIN, HIGH);
      land();
      stopMotors();
      tripleBlink();
      break;
    
    case TESTING: // Just read values and send them to Saleae
      digitalWrite(LED_PIN, HIGH);
      readValues();
      sendReadings();
      delay(150);
      break;
  }
}

/* ----- SETUP FUNCTIONS ----- */
void setupPins(){
  pinMode(DRONE_POWER, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PC13, HIGH); // LED off
  tripleBlink();
}

void setupServos(){
  motor_init();
  
  sx.attach(PA0);
  sy.attach(PA1);
  sz.attach(PA2);
  sdist.attach(PA3);
  spres.attach(PA6);
}

void setupSensors(){
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin(); // SCL = PB6, SDA = PB7

  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  bool lidarStart = lidar.begin(0x29, &Wire);
  lidar.startRanging();

  bool dpsStart = dps.begin_I2C(0x77);
  dps.configurePressure(DPS310_64HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_16SAMPLES);
}

void setupRC(){
  remote_control_init(&msg);
}

/* ----- SENSOR READING FUNCTIONS ----- */
void readValues(){ // Read all sensors
  readGyro();
  readLidar();
  readPressure();
}

void readGyro(){ // Read from Gyro
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  accel_x_g = ax / TWO_GS_FORCE;
  accel_y_g = ay / TWO_GS_FORCE;
  accel_z_g = az / TWO_GS_FORCE;

  gyro_x_dps = gx / GYRO_DPS;
  gyro_y_dps = gy / GYRO_DPS;
  gyro_z_dps = gz / GYRO_DPS;

  // Constrain to ±1g range
  // accel_x_g = constrain(accel_x_g, -1.0, 1.0);
  // accel_y_g = constrain(accel_y_g, -1.0, 1.0);
  // accel_z_g = constrain(accel_z_g, -1.0, 1.0);
}

void readLidar(){ // Read from Lidar TOF sensor
  if (lidar.dataReady()) {
    distance = lidar.distance(); // Distance in millimeters
    // if (distance != -1) {
    //   Serial.print("Distance: ");
    //   Serial.print(distance);
    //   Serial.println(" mm");
    // } else {
    //   Serial.println("Distance Out of range");
    // }

    lidar.clearInterrupt(); // Reset data ready flag
  }
}

void readPressure(){ // Read from pressure sensor
  dps.getEvents(&temp, &pressure);
}

void sendReadings(){ // Send readings to Saleae
  // Reading on Saleae
  pwmx = 1500 + (accel_x_g * 500);  // -1g → 1000, 0g → 1500, +1g → 2000
  pwmy = 1500 + (accel_y_g * 500);
  pwmz = 1000 + (accel_z_g * 500);
  pwmdist = 1000 + (distance/4);
  pwmpres = pressure.pressure + 200; // 857.81

  sx.writeMicroseconds(pwmx);
  sy.writeMicroseconds(pwmy);
  sz.writeMicroseconds(pwmz);
  sdist.writeMicroseconds(pwmdist);
  spres.writeMicroseconds(pwmpres);
}

/* ----- MOTOR CONTROL FUNCTIONS ----- */
void land(){ // Landing sequence
  readPressure();
  static float curPressure = pressure.pressure;
  while((getSpeed(1) + getSpeed(2) + getSpeed(3) + getSpeed(4)) > 4800) {
    readPressure();
    if(pressure.pressure < curPressure - PRESSURE_THRESHOLD){ // if drone is falling, don't adjust motors
      curPressure = pressure.pressure;
    } else{ // if drone is not falling, decrease motor speed
      changeSpeed(-1);
      writeESCs();
    }
    delay(80);
  }
  for (int pwm = 1150; pwm >= STOP_SPEED; pwm -= 5) {
    setSpeed(pwm);
    writeESCs();
    delay(80);
  }
}

void forceLand(){ // force the drone to land if pressure sensor isn't working
  while((getSpeed(1) + getSpeed(2) + getSpeed(3) + getSpeed(4)) > 4800) {
    changeSpeed(-1);
    writeESCs();
    delay(80);
  }
  for (int pwm = 1150; pwm >= STOP_SPEED; pwm -= 5) {
    setSpeed(pwm);
    writeESCs();
    delay(80);
  }
}

/* ----- RC Functions ----- */
void rcISR(){
  moveX(msg.x_change);
  moveY(msg.y_change);
  hoverPressure += msg.z_change;
}
/* ----- Testing Functions ----- */

/* ----- Additional Functions -----*/
void tripleBlink(){ // Blinks the onboard LED 3x
  for(int i=0;i<3;i++){
    digitalWrite(PC13, LOW);
    delay(200);
    digitalWrite(PC13, HIGH);
    delay(200);
  }
}