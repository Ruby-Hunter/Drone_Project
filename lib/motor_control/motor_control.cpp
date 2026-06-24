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
  float pitch = sHistory.accel_x_g[sHistory.index];
  pitch = constrain(pitch, -1.5, 1.5); // Limit pitch to prevent extreme motor changes
  if(pitch > (0 + MOTION_THRESHOLD)){ // if it pitches too far forwards, increase front motor speeds
    proportional(motor_1_Speed, pitch, 0, P_PITCH, 1);
    proportional(motor_2_Speed, pitch, 0, P_PITCH, 1);
    integral(motor_1_Speed, sHistory.accel_x_g, 0, I_PITCH, 0);
    integral(motor_2_Speed, sHistory.accel_x_g, 0, I_PITCH, 0);
    derivative(motor_1_Speed, pitch, sHistory.accel_x_g[sHistory.prev_index], D_PITCH, 0);
    derivative(motor_2_Speed, pitch, sHistory.accel_x_g[sHistory.prev_index], D_PITCH, 0);
  }
  else if(pitch < (0 - MOTION_THRESHOLD)){ // pitch too far backwards, increase back motor speeds
    proportional(motor_3_Speed, pitch, 0, P_PITCH, 1);
    proportional(motor_4_Speed, pitch, 0, P_PITCH, 1);
    integral(motor_3_Speed, sHistory.accel_x_g, 0, I_PITCH, 0);
    integral(motor_4_Speed, sHistory.accel_x_g, 0, I_PITCH, 0);
    derivative(motor_3_Speed, pitch, sHistory.accel_x_g[sHistory.prev_index], D_PITCH, 0);
    derivative(motor_4_Speed, pitch, sHistory.accel_x_g[sHistory.prev_index], D_PITCH, 0);
  }
}

void balanceRoll(SensorDataHistory sHistory){
  // TODO: Tune PID constants, set desired value to variable
  float roll = sHistory.accel_y_g[sHistory.index];
  roll = constrain(roll, -1.5, 1.5); // Limit roll to prevent extreme motor changes
  if(roll > (0 + MOTION_THRESHOLD)){ // if it rolls too far right, increase right (2, 3) motor speeds
    proportional(motor_2_Speed, roll, 0, P_ROLL, 1);
    proportional(motor_3_Speed, roll, 0, P_ROLL, 1);
    integral(motor_2_Speed, sHistory.accel_y_g, 0, I_ROLL, 0);
    integral(motor_3_Speed, sHistory.accel_y_g, 0, I_ROLL, 0);
    derivative(motor_2_Speed, roll, sHistory.accel_y_g[sHistory.prev_index], D_ROLL, 0);
    derivative(motor_3_Speed, roll, sHistory.accel_y_g[sHistory.prev_index], D_ROLL, 0);
  }
  else if(roll < (0 - MOTION_THRESHOLD)){ // roll too far left, increase left (1, 4) motor speeds
    proportional(motor_1_Speed, roll, 0, P_ROLL, 1);
    proportional(motor_4_Speed, roll, 0, P_ROLL, 1);
    integral(motor_1_Speed, sHistory.accel_y_g, 0, I_ROLL, 0);
    integral(motor_4_Speed, sHistory.accel_y_g, 0, I_ROLL, 0);
    derivative(motor_1_Speed, roll, sHistory.accel_y_g[sHistory.prev_index], D_ROLL, 0);
    derivative(motor_4_Speed, roll, sHistory.accel_y_g[sHistory.prev_index], D_ROLL, 0);
  }
}

void balanceAltitude(SensorDataHistory sHistory, float hoverPressure){
  // TODO: Tune PID constants, set desired value to variable
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
  // TODO: Tune PID constants, set desired value to variable
  uint16_t lidar_distance_mm = sHistory.lidar_distance_mm[sHistory.index];
  uint16_t speed_change_u = 100; // Give it a buffer so we don't have to change PID plant type
  proportional(speed_change_u, lidar_distance_mm, desiredHeight_mm, P_ALTITUDE, 1);
  integral(speed_change_u, (float *)sHistory.lidar_distance_mm, 0, I_ALTITUDE, 0);
  derivative(speed_change_u, lidar_distance_mm, sHistory.lidar_distance_mm[sHistory.prev_index], D_ALTITUDE, 0);
  int16_t speed_change = speed_change_u - 100;
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

bool motorSpeedsEqual(uint16_t speed) {
  return getSpeed(1) == getSpeed(2) == getSpeed(3) == getSpeed(4) == speed;
}