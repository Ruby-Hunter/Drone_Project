#include <Servo.h>
#include <cmath>
#include <cstring>
#include <Arduino.h>
#include "motor_control.h"
#include "remote_control.h"
#include "sensors.h"
#include "config.h"

Servo sx, sy, sz, sdist, spres; // Saleae testing

SensorData sData; // sensor data
float basePressure, hoverPressure;
float baseAltitude, altitude;

// For Sending to Saleae
int16_t pwmx, pwmy, pwmz, pwmdist, pwmpres;

// State Machine states
enum State { OFF, INIT, HOVERING, FLYING, LANDING, TESTING };
uint8_t currentState = TESTING;

msgData msg;

/* ----- FUNCTION DECLARATIONS ----- */
void setupPins();
void setupServos();
void setupRC();
void setupSerial();
void stateMachine();
void sendReadings();
void land();
void forceLand();
void rcISR();
void tripleBlink();

void setup() {
  setupSerial();
  setupPins();
  setupServos();
  setupSensors();
  // setupRC();
  tripleBlink();
  delay(1000);
}

void loop() {
  stateMachine();
  delay(MS_DELAY);
  Serial.println("USB serial is working");
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
      if(sData.distance_mm < LANDING_DISTANCE_MM){
        currentState = LANDING;
      } else if(!isStable(msg)){
        currentState = FLYING;
      }
      break;
    
    case FLYING:
      if(sData.distance_mm < LANDING_DISTANCE_MM){
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
      readPressure(sData);
      basePressure = sData.pressure.pressure; // pressure.altitude?
      baseAltitude = sData.pressure.altitude;
      takeOff();
      readPressure(sData);
      hoverPressure = sData.pressure.pressure;
      altitude = sData.pressure.altitude;
      break;
    
    case HOVERING:
      readValues(sData);
      sendReadings();

      balancePitch(sData.accel_x_g, sData.gyro_x_dps);
      balanceRoll(sData.accel_y_g, sData.gyro_y_dps);
      balanceAltitude(sData.pressure.pressure, hoverPressure);

      writeESCs();
      break;
    
    case FLYING:
      readValues(sData);
      sendReadings();

      if(xStable(msg)) balancePitch(sData.accel_x_g, sData.gyro_x_dps); // Keep balanced if no move command, else move
      else moveX(msg.x_change);
      if(yStable(msg)) balanceRoll(sData.accel_y_g, sData.gyro_y_dps);
      else moveY(msg.y_change);
      balanceAltitude(sData.pressure.pressure, hoverPressure);

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
      readValues(sData);
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



void setupRC(){
  remote_control_init(&msg);
}

void setupSerial(){
  delay(2000);
  Serial.begin(115200);
  Serial.println("USB CDC working");
}

/* ----- SENSOR READING FUNCTIONS ----- */
void sendReadings(){ // Send readings to Saleae
  // Reading on Saleae
  pwmx = 1500 + (sData.accel_x_g * 500);  // -1g → 1000, 0g → 1500, +1g → 2000
  pwmy = 1500 + (sData.accel_y_g * 500);
  pwmz = 1000 + (sData.accel_z_g * 500);
  pwmdist = 1000 + (sData.distance_mm/4);
  pwmpres = sData.pressure.pressure + 200; // 857.81

  sx.writeMicroseconds(pwmx);
  sy.writeMicroseconds(pwmy);
  sz.writeMicroseconds(pwmz);
  sdist.writeMicroseconds(pwmdist);
  spres.writeMicroseconds(pwmpres);
}

/* ----- MOTOR CONTROL FUNCTIONS ----- */
void land(){ // Landing sequence
  readPressure(sData);
  static float curPressure = sData.pressure.pressure;
  while((getSpeed(1) + getSpeed(2) + getSpeed(3) + getSpeed(4)) > 4800) {
    readPressure(sData);
    if(sData.pressure.pressure < curPressure - PRESSURE_THRESHOLD){ // if drone is falling, don't adjust motors
      curPressure = sData.pressure.pressure;
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