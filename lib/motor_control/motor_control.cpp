#include "motor_control.h"
#include "config.h"
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

void stopMotors(){
  setSpeed(STOP_SPEED);
  writeESCs();
}

void balancePitch(float accel_x_g){
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

void balanceRoll(float accel_y_g){
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

void balanceAltitude(float pressure, float hoverPressure){
  if(pressure < (hoverPressure - PRESSURE_THRESHOLD)){ // drone is falling
    changeSpeed(3);
  }
  else if(pressure > (hoverPressure + PRESSURE_THRESHOLD)){ // drone is rising
    changeSpeed(-3);
  }
}

void takeOff(){
  
  for (int pwm = STOP_SPEED; pwm <= START_SPEED; pwm += (5 + sqrt(START_SPEED - pwm))) {
    setSpeed(pwm);
    writeESCs();
    delay(50);
  }
  
  digitalWrite(LED_PIN, LOW);
}

// void land(){

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