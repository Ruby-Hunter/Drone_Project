#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_DPS310.h>
#include <cmath>

#define MOTOR1 PA8
#define MOTOR2 PA9
#define MOTOR3 PA10
#define MOTOR4 PA11
#define DRONE_POWER PB4
#define LED_PIN PC13

const uint32_t MS_DELAY = 50;
const float MOTION_THRESHOLD = 0.2;  // Threshold for motion detection (G's)
const float PRESSURE_THRESHOLD = 5; // In hPa
const float TWO_GS_FORCE = 16384.0f;
const float GYRO_DPS = 131.0f; // Divide raw data by this for Degrees Per Second (DPS)
const uint16_t STOP_SPEED = 1000;
const uint16_t START_SPEED = 1320;
const uint16_t RISING_HOVER_SPEED = 1360;
const uint16_t MAX_HOVER_SPEED = 1550; // increase if needed
const uint16_t MAX_SPEED = 1550;
const int16_t LANDING_DISTANCE_MM = 200;

Servo esc1, esc2, esc3, esc4; // Motors
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

uint16_t motor_1_Speed = STOP_SPEED; //Forward left
uint16_t motor_2_Speed = STOP_SPEED; //Forward right
uint16_t motor_3_Speed = STOP_SPEED; //Back right
uint16_t motor_4_Speed = STOP_SPEED; //Back left

enum State { OFF, INIT, HOVERING, LANDING, TESTING };
uint8_t currentState = TESTING;

void setup() {
  setupPins();
  setupServos();
  setupSensors();
  tripleBlink();
  delay(1000);
}

void loop() {
  stateMachine();
  delay(MS_DELAY);
}

void stateMachine(){
  if(!digitalRead(DRONE_POWER)){
    currentState = OFF;
    stopMotors();
  }

  switch (currentState){
    case OFF:
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      if(digitalRead(DRONE_POWER) == HIGH){
        currentState = INIT;
      }
      break;
    
    case INIT:
      tripleBlink();
      delay(3000);  // Wait for ESCs to initialize
      takeOff();
      currentState = HOVERING;
      break;
    
    case HOVERING:
      readValues();
      if(distance < LANDING_DISTANCE_MM){
        currentState = LANDING;
        break;
      }
      sendReadings();
      balancePitch();
      balanceRoll();
      balanceAltitude();
      writeESCs();
      break;

    case LANDING:
      digitalWrite(LED_PIN, HIGH);
      land();
      stopMotors();
      tripleBlink();
      currentState = TESTING;
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
  esc1.attach(MOTOR1);
  esc2.attach(MOTOR2);
  esc3.attach(MOTOR3);
  esc4.attach(MOTOR4);
  
  sx.attach(PA0);
  sy.attach(PA1);
  sz.attach(PA2);
  sdist.attach(PA3);
  spres.attach(PA6);

  stopMotors();
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

/* ----- FLIGHT CONTROL FUNCTIONS ----- */
void setSpeed(uint16_t newSpeed){ // Sets speed of all motors to newSpeed
  motor_1_Speed = newSpeed;
  motor_2_Speed = newSpeed;
  motor_3_Speed = newSpeed;
  motor_4_Speed = newSpeed;
}

void changeSpeed(int16_t change){ // Adjusts all motor speeds by change
  motor_1_Speed += change;
  motor_2_Speed += change;
  motor_3_Speed += change;
  motor_4_Speed += change;
}

void writeESCs(){ // Writes new PWM values to motors
  motor_1_Speed = constrain(motor_1_Speed, STOP_SPEED, MAX_SPEED);
  motor_2_Speed = constrain(motor_2_Speed, STOP_SPEED, MAX_SPEED);
  motor_3_Speed = constrain(motor_3_Speed, STOP_SPEED, MAX_SPEED);
  motor_4_Speed = constrain(motor_4_Speed, STOP_SPEED, MAX_SPEED);
  esc1.writeMicroseconds(motor_1_Speed);
  esc2.writeMicroseconds(motor_2_Speed);
  esc3.writeMicroseconds(motor_3_Speed);
  esc4.writeMicroseconds(motor_4_Speed);
}

void balancePitch(){ // Balances motors based on pitch
  if(accel_x_g > (0 + MOTION_THRESHOLD)){ // if it pitches too far forwards, increase front motor speeds
    int8_t change = map(accel_x_g, -1, 1, -10, 10);
    motor_1_Speed += 1 + change;
    motor_2_Speed += 1 + change;
  }
  else if(accel_x_g < (0 - MOTION_THRESHOLD)){ // pitch too far backwards, increase back motor speeds
    int8_t change = map(-accel_x_g, -1, 1, -10, 10); // accel_x_g is negative here
    motor_3_Speed += 1 + change;
    motor_4_Speed += 1 + change;
  }
}

void balanceRoll(){ // Balances motors based on roll
  if(accel_y_g > (0 + MOTION_THRESHOLD)){
    int8_t change = map(accel_y_g, -1, 1, -10, 10);
    motor_2_Speed += 1 + change;
    motor_3_Speed += 1 + change;
    // if (motor_2_Speed >= MAX_HOVER_SPEED || motor_3_Speed >= MAX_HOVER_SPEED){
    //   motor_1_Speed -= 1 + change;
    //   motor_4_Speed -= 1 + change;
    // }
  }
  else if(accel_y_g < (0 - MOTION_THRESHOLD)){
    int8_t change = map(-accel_y_g, -1, 1, -10, 10);
    motor_1_Speed += 1 + change;
    motor_4_Speed += 1 + change;
  }
}

void balanceAltitude(){ // Checks for changes in verticle position, adjusts all motors based on it
  // if(accel_z_g > (0 + MOTION_THRESHOLD)){ // Checks vertical acceleration
  //   int change = map(accel_z_g, -1, 1, -10, 10);
  //   motor_1_Speed += 1 + change;
  //   motor_2_Speed += 1 + change;
  //   motor_3_Speed += 1 + change;
  //   motor_4_Speed += 1 + change;
  // }
  // else if(accel_z_g < (0 - MOTION_THRESHOLD)){ // Reads the change in vertical acceleration
  //   int change = map(accel_z_g, -1, 1, -10, 10);
  //   motor_1_Speed += 1 - change;
  //   motor_2_Speed += 1 - change;
  //   motor_3_Speed += 1 - change;
  //   motor_4_Speed += 1 - change;
  // }

  //TEST CODE
  // if(motor_1_Speed > RISING_HOVER_SPEED && motor_2_Speed > RISING_HOVER_SPEED && motor_3_Speed > RISING_HOVER_SPEED && motor_4_Speed > RISING_HOVER_SPEED){
  //   motor_1_Speed -= 5;
  //   motor_2_Speed -= 5;
  //   motor_3_Speed -= 5;
  //   motor_4_Speed -= 5;
  // }

  if(pressure.pressure < (hoverPressure - PRESSURE_THRESHOLD)){ // drone is falling
    changeSpeed(3);
  }
  else if(pressure.pressure > (hoverPressure + PRESSURE_THRESHOLD)){ // drone is rising
    changeSpeed(-3);
  }
}

void takeOff(){ // Takeoff sequence
  readPressure();
  basePressure = pressure.pressure; // pressure.altitude?
  for (int pwm = STOP_SPEED; pwm <= START_SPEED; pwm += (5 + sqrt(START_SPEED - pwm))) {
    setSpeed(pwm);
    writeESCs();
    delay(50);
  }
  readPressure();
  hoverPressure = pressure.pressure;
  digitalWrite(LED_PIN, LOW);
}

void land(){ // Landing sequence
  readPressure();
  static float curPressure = pressure.pressure;
  while((motor_1_Speed + motor_2_Speed + motor_3_Speed + motor_4_Speed) > 4800) {
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
  while((motor_1_Speed + motor_2_Speed + motor_3_Speed + motor_4_Speed) > 4800) {
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

void stopMotors(){ // Sets all motors to 1000
  setSpeed(STOP_SPEED);
  writeESCs();
}

/* ----- Testing Functions ----- */
uint16_t getSpeed(int motorChoice = 4){ // Gets speed of motorChoice motor; default is 4
  switch (motorChoice) {
    case 1:
      return motor_1_Speed;
      break;
    case 2:
      return motor_2_Speed;
      break;
    case 3:
      return motor_3_Speed;
      break;
    default:
      return motor_4_Speed;
      break;
  }
}

/* ----- Additional Functions -----*/
void tripleBlink(){ // Blinks the onboard LED 3x
  for(int i=0;i<3;i++){
    digitalWrite(PC13, LOW);
    delay(200);
    digitalWrite(PC13, HIGH);
    delay(200);
  }
}