#include "motor_control.h"
#include "config.h"
#include "utils.h"
#include <Servo.h>
#include <Arduino.h>
#include <cmath>

Servo esc1, esc2, esc3, esc4; // Motors

static uint16_t motor_1_Speed; //Forward left
static uint16_t motor_2_Speed; //Forward right
static uint16_t motor_3_Speed; //Back right
static uint16_t motor_4_Speed; //Back left

void motor_init(){
  esc1.attach(MOTOR1);
  esc2.attach(MOTOR2);
  esc3.attach(MOTOR3);
  esc4.attach(MOTOR4);
  setSpeed(STOP_SPEED);
  writeESCs();
}

void setSpeed(uint16_t newSpeed){
  motor_1_Speed = newSpeed;
  motor_2_Speed = newSpeed;
  motor_3_Speed = newSpeed;
  motor_4_Speed = newSpeed;
}

void changeSpeed(int16_t change){
  motor_1_Speed += change;
  motor_2_Speed += change;
  motor_3_Speed += change;
  motor_4_Speed += change;
}

void moveX(int8_t x_movement){
  static int8_t x_distance = 0;
  int8_t delta = x_movement - x_distance;
  if(abs(delta) > MOVEMENT_THRESHOLD){
    motor_1_Speed += delta;
    motor_2_Speed += delta;
    motor_3_Speed -= delta;
    motor_4_Speed -= delta;
    x_distance = x_movement;
  }
}

void moveY(int8_t y_movement){
  static int8_t y_distance = 0;
  int8_t delta = y_movement - y_distance;
  if(abs(delta) > MOVEMENT_THRESHOLD){
    motor_2_Speed += delta;
    motor_3_Speed += delta;
    motor_1_Speed -= delta;
    motor_4_Speed -= delta;
    y_distance = y_movement;
  }
}

void moveZ(int8_t z_movement){ //TODO: Move implementation into here
  
}

void writeESCs(){
  motor_1_Speed = constrain(motor_1_Speed, STOP_SPEED, MAX_SPEED);
  motor_2_Speed = constrain(motor_2_Speed, STOP_SPEED, MAX_SPEED);
  motor_3_Speed = constrain(motor_3_Speed, STOP_SPEED, MAX_SPEED);
  motor_4_Speed = constrain(motor_4_Speed, STOP_SPEED, MAX_SPEED);
  // Serial.print("Motor Speeds: "); Serial.print(motor_1_Speed); Serial.print(", "); Serial.print(motor_2_Speed); Serial.print(", "); Serial.print(motor_3_Speed); Serial.print(", "); Serial.println(motor_4_Speed);
  esc1.writeMicroseconds(motor_1_Speed);
  esc2.writeMicroseconds(motor_2_Speed);
  esc3.writeMicroseconds(motor_3_Speed);
  esc4.writeMicroseconds(motor_4_Speed);
}

void printSpeeds(){
  Serial.print("Motor Speeds: "); Serial.print(motor_1_Speed); 
  Serial.print(", "); Serial.print(motor_2_Speed);
  Serial.print(", "); Serial.print(motor_3_Speed);
  Serial.print(", "); Serial.println(motor_4_Speed);
}

void stopMotors(){
  setSpeed(STOP_SPEED);
  writeESCs();
}

void balancePitch(SensorDataHistory sHistory){
  // TODO: Tune PID constants, set desired value to variable
  float pitch = sHistory.accel_x_g[sHistory.imu_idx];
  pitch = constrain(pitch, -1.5, 1.5); // Limit pitch to prevent extreme motor changes

  uint16_t speed_change_u = 100;
  proportional(speed_change_u, pitch, DESIRED_PITCH, P_PITCH, 1);
  integral(speed_change_u, sHistory.accel_x_g, DESIRED_PITCH, I_PITCH, 1);
  derivative(speed_change_u, pitch, sHistory.accel_x_g[sHistory.prev_imu_idx], DESIRED_PITCH, D_PITCH, 1);
  int16_t speed_change = 100 - speed_change_u; // Speed change is neg when it should be pos
  speed_change = constrain(speed_change, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);

  // Serial.print("Pitch: "); Serial.print(pitch); Serial.print(", Speed Change: "); Serial.println(speed_change);

  // if(pitch > (0 + MOTION_THRESHOLD)){ // if it pitches too far forwards, increase front motor speeds
    motor_1_Speed += speed_change;
    motor_2_Speed += speed_change;
    motor_3_Speed -= speed_change;
    motor_4_Speed -= speed_change;
  // }
  // else if(pitch < (0 - MOTION_THRESHOLD)){ // pitch too far backwards, increase back motor speeds
  //   motor_3_Speed += speed_change;
  //   motor_4_Speed += speed_change;
  //   motor_1_Speed -= speed_change / 2;
  //   motor_2_Speed -= speed_change / 2;
  // }
}

void balanceRoll(SensorDataHistory sHistory){
  // TODO: Tune PID constants, set desired value to variable
  float roll = sHistory.accel_y_g[sHistory.imu_idx];
  roll = constrain(roll, -1.5, 1.5); // Limit roll to prevent extreme motor changes

  uint16_t speed_change_u = 100;
  proportional(speed_change_u, roll, DESIRED_ROLL, P_ROLL, 1);
  integral(speed_change_u, sHistory.accel_y_g, DESIRED_ROLL, I_ROLL, 1);
  derivative(speed_change_u, roll, sHistory.accel_y_g[sHistory.prev_imu_idx], DESIRED_ROLL, D_ROLL, 1);
  int16_t speed_change = 100 - speed_change_u; // Speed change is neg when it should be pos
  speed_change = constrain(speed_change, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);

  // Serial.print("Roll: "); Serial.print(roll); Serial.print(", Speed Change: "); Serial.println(speed_change);

  // if(roll > (0 + MOTION_THRESHOLD)){ // if it rolls too far right, increase right (2, 3) motor speeds
    motor_2_Speed += speed_change;
    motor_3_Speed += speed_change;
    motor_1_Speed -= speed_change;
    motor_4_Speed -= speed_change;
  // }
  // else if(roll < (0 - MOTION_THRESHOLD)){ // roll too far left, increase left (1, 4) motor speeds
  //   motor_1_Speed += speed_change;
  //   motor_4_Speed += speed_change;
  //   motor_2_Speed -= speed_change / 2;
  //   motor_3_Speed -= speed_change / 2;
  // }
}

void balanceAltitude(SensorDataHistory sHistory, float hoverPressure){
  proportional(motor_1_Speed, sHistory.pressure[sHistory.index].pressure, hoverPressure, P_ALTITUDE, 1);
  // int16_t pressureError = hoverPressure - pressure;
  // if(pressure < (hoverPressure - PRESSURE_THRESHOLD_HPA)){ // drone is falling
  //   changeSpeed(map(pressureError, 
  //                   0, hoverPressure + (ALTITUDE_THRESHOLD_MM*5)*PRESSURE_CHANGE_TO_ALTITUDE_MM, 
  //                   1, MAX_PWM_CHANGE/2)); // Proportional control based on distance from target height
  // }
  // else if(pressure > (hoverPressure + PRESSURE_THRESHOLD_HPA)){ // drone is rising
  //   changeSpeed(map(pressureError, 
  //                   -hoverPressure + (ALTITUDE_THRESHOLD_MM*5)*PRESSURE_CHANGE_TO_ALTITUDE_MM, 0, 
  //                   -MAX_PWM_CHANGE/2, -1)); // Proportional control based on distance from target height
  // }

  /* Method 2: Check exact altitude relative to ground */

}

void balanceAltitudeLidar(SensorDataHistory sHistory, int16_t desiredHeight_mm){
  // TODO: Tune PID constants
  uint16_t lidar_distance_mm = sHistory.lidar_distance_mm[sHistory.index];
  // lidar_distance_mm = correctAltitude(lidar_distance_mm, sHistory.accel_x_g[sHistory.imu_idx], sHistory.accel_y_g[sHistory.imu_idx]);
  uint16_t speed_change_u = 100; // Give it a buffer so we don't have to change PID plant type
  proportional(speed_change_u, lidar_distance_mm, desiredHeight_mm, P_ALTITUDE, 1);
  integral(speed_change_u, sHistory.lidar_distance_mm, desiredHeight_mm, I_ALTITUDE, 0);
  derivative(speed_change_u, lidar_distance_mm, sHistory.lidar_distance_mm[sHistory.prev_index], desiredHeight_mm, D_ALTITUDE, 0);
  int16_t speed_change = speed_change_u - 100; // Keep this, it works right here
  speed_change = sqrt(abs(speed_change)) * (speed_change < 0 ? -1 : 1); // Make it a square root function
  speed_change = constrain(speed_change, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
  changeSpeed(speed_change);
  
  // int16_t height_diff_mm = sHistory.lidar_distance_mm[sHistory.index] - desiredHeight_mm;

  // if(sHistory.lidar_distance_mm[sHistory.index] < (desiredHeight_mm - ALTITUDE_THRESHOLD_MM)){
  //   changeSpeed(map(height_diff_mm, 0, height_diff_mm*10, 1, 3));
  // }
  // else if(sHistory.lidar_distance_mm[sHistory.index] > (desiredHeight_mm + ALTITUDE_THRESHOLD_MM)){
  //   changeSpeed(map(height_diff_mm, -height_diff_mm*10, 0, -3, -1));
  // }
}

void takeOff(int16_t distance_mm){
  // Gradually increase speed based on distance from target height
  static int16_t lastError = TAKEOFF_HEIGHT_MM;
  uint16_t error = TAKEOFF_HEIGHT_MM - distance_mm;

  // Proportional control based on distance from target height
  int16_t change = TAKEOFF_SPEED * ((error)/TAKEOFF_HEIGHT_MM) + 2;
  // Derivative control based on change in distance from target height
  change += map(error - lastError, -TAKEOFF_HEIGHT_MM, TAKEOFF_HEIGHT_MM, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);

  lastError = error;
  changeSpeed(change);
}

void forceLand(){ // Force the drone to land if pressure sensor isn't working
  static uint16_t pwm = 1350;
  if(!motorSpeedsLessThan(1200)) {
    changeSpeed(-3);
    writeESCs();
  }
  else {
    if(pwm <= 1200) Serial.println("--- Hit bottom ---"); // debug
    setSpeed(pwm);
    writeESCs();
    if(pwm > STOP_SPEED){
      pwm -= 5;
    } else {
      Serial.println("-- Landed --");
      stopMotors();
    }
  }
}

// void land(){
// }

// void forceLand(){
// }

uint16_t getSpeed(int motorChoice){
  switch (motorChoice) {
    case 1:
      return motor_1_Speed;
    case 2:
      return motor_2_Speed;
    case 3:
      return motor_3_Speed;
    default:
      return motor_4_Speed;
  }
}

void constrain_hover_pwm(){
  motor_1_Speed = constrain(motor_1_Speed, MIN_HOVER_SPEED, MAX_HOVER_SPEED);
  motor_2_Speed = constrain(motor_2_Speed, MIN_HOVER_SPEED, MAX_HOVER_SPEED);
  motor_3_Speed = constrain(motor_3_Speed, MIN_HOVER_SPEED, MAX_HOVER_SPEED);
  motor_4_Speed = constrain(motor_4_Speed, MIN_HOVER_SPEED, MAX_HOVER_SPEED);
}

bool motorSpeedsEqual(uint16_t speed) {
  return getSpeed(1) == getSpeed(2) 
    && getSpeed(2) == getSpeed(3) 
    && getSpeed(3) == getSpeed(4) 
    && getSpeed(4) == speed;
}

bool motorSpeedsLessThan(uint16_t speed) {
  return getSpeed(1) < speed
    && getSpeed(2) < speed
    && getSpeed(3) < speed
    && getSpeed(4) < speed;
}

uint16_t getMotorSpeedAverage() {
  return (getSpeed(1) + getSpeed(2) + getSpeed(3) + getSpeed(4)) / 4;
}