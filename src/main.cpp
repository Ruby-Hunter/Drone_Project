/*
Filename: main.cpp
Desc:     Runs the main program, setting everything up and then ticking a SyncSM
          every DELAY_MS ms. 
*/

#include <Servo.h>
#include <cmath>
#include <cstring>
#include <Arduino.h>
#include "motor_control.h"
#include "remote_control.h"
#include "sensors.h"
#include "config.h"
#include "utils.h"

SensorData sData; // Most recent sensor data
SensorDataHistory sDataHist; // History of sensor data for PID control

float basePressure, hoverPressure;
float altitude;

// For Sending to Saleae
int16_t pwmx, pwmy, pwmz, pwmdist, pwmpres;

// State Machine states
enum State { OFF, INIT, STARTING, TAKEOFF, HOVERING, FLYING, LANDING, TESTING };
uint8_t currentState = OFF;
uint32_t lastTick = 0;
uint8_t power_off_count = 0; // If read 10 consecutive power offs, switch to OFF state
uint8_t takeoff_count = 0; // If sensors are stable STARTUP_TIME_MS/DELAY_MS times, takeoff

msgData msg;

/* ----- FUNCTION DECLARATIONS ----- */
void setupPins();
void setupServos();
void setupRC();
void setupSerial();
void stateChanges();
void stateActions();
void stateMachine();
bool reachedTakeoffHeight();
bool outsideHoveringHeight();
bool landSignal();
void readSensorData();
void sendReadings();
void land();
void forceLand();
void rcISR();
void tripleBlink();
void fiveBlink();


void setup() {
  setupPins();
  setupSerial();
  setupServos();
  setupSensors();
  // setupRC();
  Serial.println("Setup complete");
  tripleBlink();
  delay(1000);
}

void loop() { // Main loop runs every DELAY_MS ms
  if(millis() - lastTick >= DELAY_MS){
    lastTick = millis();
    stateMachine();
    // Serial.println("Tick");
  }
}

/* ----- SETUP FUNCTIONS ----- */
void setupPins(){
  pinMode(DRONE_POWER, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PC13, HIGH); // LED off
}

void setupServos(){
  motor_init();
  Serial.println("Servos initialized");
}

void setupRC(){
  remote_control_init(&msg);
}

void setupSerial(){
  Serial.begin(115200);
  delay(3000);
  if(Serial){
    Serial.println("USB CDC initialized");
    tripleBlink();
    delay(500);
    tripleBlink();
  }
  Serial.println("USB CDC setup complete");
}


/* ----- STATE MACHINE FUNCTIONS ----- */
void stateMachine(){ // State machine for drone
  if(!digitalRead(DRONE_POWER)){ // Failsafe
    power_off_count++;
    if(power_off_count > 10){
      Serial.println("=========== Power off detected ===========");
      if(currentState != OFF && currentState != TESTING){
        currentState = OFF;
        stopMotors();
      }
    }
  } else{
    power_off_count = 0;
  }

  stateChanges();

  stateActions();
}

void stateChanges(){
    switch (currentState){
    case OFF: 
      if(digitalRead(DRONE_POWER)){
        currentState = INIT;
      }
      break;

    case INIT:
      currentState = STARTING;
      break;
    
    case STARTING:
      if(readingsStable(sData)){
        // currentState = TAKEOFF;
        // Serial.println("+ Readings Stable, ready for takeoff");
        if(++takeoff_count >= STARTUP_TIME_MS/DELAY_MS){ // If readings have been stable for 3 seconds, takeoff
          Serial.println("+ Readings Stable, ready for takeoff");
          currentState = TAKEOFF;
        }
      } else {
        takeoff_count = 0;
      }
      break;
    
    case TAKEOFF:
      if(reachedTakeoffHeight()){
        currentState = HOVERING;
      }
      break;

    case HOVERING:
      if(outsideHoveringHeight()){ 
        // If drone is close to ground, too high, or pressure sensor is malfunctioning, land
        currentState = LANDING;
      } 
      // else if(!isStable(msg)){
      //   currentState = FLYING;
      // }
      break;
     
    case FLYING:
      if(landSignal()){
        currentState = LANDING;
      } else if(isStable(msg)){
        currentState = HOVERING;
      }
      break;
    
    case LANDING:
      if(motorSpeedsEqual(STOP_SPEED)){
        currentState = TESTING;
      }
      break;
  }

}

void stateActions(){
  switch (currentState){ 
    case OFF:
      Serial.println("--- Motors off ---");
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      break;
    
    case INIT: //TODO: don't wait in this case statement
      Serial.println("--- Starting up ---");
      stopMotors();
      fiveBlink();
      delay(STARTUP_TIME_MS);  // Wait for ESCs to initialize

      readPressure(sData);
      basePressure = sData.pressure.pressure;
      hoverPressure = basePressure + TAKEOFF_HEIGHT_MM * PRESSURE_CHANGE_TO_ALTITUDE_MM; // Set to hover to 1m above base
      break;
    
    case STARTING:
      Serial.println("--- Starting ---");
      setSpeed(START_SPEED);
      writeESCs();

      readSensorData();
      sendReadings();
      break;
    
    case TAKEOFF:
      Serial.println("--- Takeoff ---");
      readSensorData();
      sendReadings();

      takeOff(sData.lidar_distance_mm);
      balancePitch(sDataHist);
      balanceRoll(sDataHist);
      writeESCs();
      break;
    
    case HOVERING:
      Serial.println("--- Hovering ---");
      readSensorData();
      sendReadings();

      balancePitch(sDataHist);
      balanceRoll(sDataHist);
      // balanceAltitude(sData.pressure.pressure, hoverPressure);
      balanceAltitudeLidar(sDataHist, TAKEOFF_HEIGHT_MM);

      writeESCs();
      break;
    
    case FLYING:
      Serial.println("--- Flying ---");
      readSensorData();
      sendReadings();

      if(xStable(msg)) balancePitch(sDataHist); // Keep balanced if no move command, else move
      else moveX(msg.x_change);
      if(yStable(msg)) balanceRoll(sDataHist);
      else moveY(msg.y_change);
      balanceAltitude(sDataHist, hoverPressure);

      writeESCs();
      break;

    case LANDING:
      Serial.println("--- Landing Sequence ---");
      digitalWrite(LED_PIN, HIGH);
      readSensorData();
      sendReadings();

      forceLand();
      balancePitch(sDataHist);
      balanceRoll(sDataHist);
      // land();
      break;
    
    case TESTING: // Just read values and print them
      Serial.println("--- Testing ---");
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      writeESCs(); // write motor speeds
      readSensorData();
      sendReadings();
      delay(500 - DELAY_MS);
      break;
  }
}

bool reachedTakeoffHeight(){ // Returns true if drone is at or above takeoff height
  return {abs(sData.lidar_distance_mm - TAKEOFF_HEIGHT_MM) <= ALTITUDE_THRESHOLD_MM};
}

bool outsideHoveringHeight(){ // Returns true if drone is outside of its min/max height
  return {sData.lidar_distance_mm < LANDING_DISTANCE_MM || sData.pressure.pressure > (HEIGHT_LIMIT_HPA + basePressure)};
}

bool landSignal(){ // Returnns true if signal to land has been recieved
  return {sData.lidar_distance_mm < LANDING_DISTANCE_MM};
}


/* ----- SENSOR READING FUNCTIONS ----- */
void readSensorData(){
  readSensors(sData);
  advanceIndex(sDataHist);
  updateHistory(sDataHist, sData);
}

void sendReadings(){ // Send readings to Operator
  printSpeeds();
  Serial.print("Accel X (g): "); Serial.print(sData.accel_x_g);
  Serial.print(" | Accel Y (g): "); Serial.print(sData.accel_y_g);
  Serial.print(" | Accel Z (g): "); Serial.print(sData.accel_z_g);
  Serial.print(" | Gyro X (dps): "); Serial.print(sData.gyro_x_dps);
  Serial.print(" | Gyro Y (dps): "); Serial.print(sData.gyro_y_dps);
  Serial.print(" | Gyro Z (dps): "); Serial.print(sData.gyro_z_dps);
  Serial.print(" | Distance (mm): "); Serial.print(sData.lidar_distance_mm);
  Serial.print(" | BasePressure (hPa): "); Serial.print(basePressure);
  Serial.print(" | Pressure (hPa): "); Serial.print(sData.pressure.pressure);
  Serial.print(" | Altitude (mm): "); Serial.println((sData.pressure.pressure - basePressure) / PRESSURE_CHANGE_TO_ALTITUDE_MM);
}


/* ----- MOTOR CONTROL FUNCTIONS ----- */
void land(){ // Landing sequence
  readSensorData();
  static float curPressure = sData.pressure.pressure;

  while((getSpeed(1) + getSpeed(2) + getSpeed(3) + getSpeed(4)) > 4800) {
    readSensorData();
    if(sData.pressure.pressure < sDataHist.pressure[sDataHist.index - 1].pressure - PRESSURE_THRESHOLD_HPA){ // if drone is falling, don't adjust motors
      continue;
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

void forceLand(){ // Force the drone to land if pressure sensor isn't working
  static uint16_t pwm = 1200;
  if((getSpeed(1) + getSpeed(2) + getSpeed(3) + getSpeed(4)) > 4800) {
    changeSpeed(-3);
    writeESCs();
  }
  else {
    if(pwm == 1200) Serial.println("--- Hit bottom ---"); // debug
    setSpeed(pwm);
    writeESCs();
    if(pwm > STOP_SPEED){
      pwm -= 6;
    } else {
      Serial.println("Landed");
      stopMotors();
      fiveBlink();
    }
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

void fiveBlink(){ // Blinks the onboard LED 5x
  for(int i=0;i<5;i++){
    digitalWrite(PC13, LOW);
    delay(150);
    digitalWrite(PC13, HIGH);
    delay(150);
  }
}

