/*
Filename: main.cpp
Desc:     Runs the main program, setting everything up and then ticking a SyncSM
          every MAIN_DELAY_MS ms. 
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

SensorData sData{}; // Most recent sensor data
SensorData averageData{};
SensorData serialData{};
SensorDataHistory sDataHist{}; // History of sensor data for PID control

float basePressure, hoverPressure;
float altitude;

// For Sending to Saleae
int16_t pwmx, pwmy, pwmz, pwmdist, pwmpres;

// State Machine states
enum State { OFF, INIT, STARTING, TAKEOFF, HOVERING, FLYING, LANDING, TESTING };
uint8_t currentState = OFF;
uint32_t lastMainTick = 0;
uint32_t lastSensorTick = 0;
uint32_t lastIMUTick = 0;
uint32_t lastSerialTick = 0;
uint8_t power_off_count = 0; // If read 10 consecutive power offs, switch to OFF state
uint8_t takeoff_count = 0; // If sensors are stable STARTUP_TIME_MS/MAIN_DELAY_MS times, takeoff

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
void sendLogs(const SensorData& data);
void sendReadings(const SensorData& data);
SensorData averageHistory(const SensorDataHistory& history, uint8_t sampleCount);
void land();
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

void loop() { // Main loop runs every MAIN_DELAY_MS ms
  if(millis() - lastMainTick >= MAIN_DELAY_MS){
    lastMainTick = millis();
    averageData = averageHistory(sDataHist, MAIN_AVERAGE_SAMPLES);
    stateMachine();
    // Serial.println("Tick");
  }

  if(millis() - lastSerialTick >= SERIAL_DELAY_MS){
    lastSerialTick = millis();
    serialData = averageHistory(sDataHist, SERIAL_AVERAGE_SAMPLES);
    sendReadings(serialData);
    // sendLogs(serialData);
  }

  if(currentState >= STARTING){
    if(millis() - lastIMUTick >= IMU_DELAY_MS){
      lastIMUTick = millis();
      readGyro(sData);
      advanceIMUIndex(sDataHist);
      updateHistory(sDataHist, sData);
    }

    if(millis() - lastSensorTick >= SENSOR_DELAY_MS){ // TODO: Check how long reading sensors takes
      lastSensorTick = millis();
      
      readSensorData();
    }
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
      // Serial.println("=========== Power off detected ===========");
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
      if(readingsStable(averageData)){
        // currentState = TAKEOFF;
        // Serial.println("+ Readings Stable, ready for takeoff");
        if(++takeoff_count >= STARTUP_TIME_MS/MAIN_DELAY_MS){ // If readings have been stable for 3 seconds, takeoff
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
      if(getMotorSpeedAverage() <= START_SPEED){
        currentState = TESTING;
        fiveBlink();
      }
      break;
  }

}

void stateActions(){
  switch (currentState){ 
    case OFF:
      // Serial.println("--- Motors off ---");
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      break;
    
    case INIT:
      Serial.println("--- INIT ---");
      stopMotors();
      fiveBlink();
      delay(STARTUP_TIME_MS);  // Wait for ESCs to initialize

      readPressure(sData);
      basePressure = sData.pressure.pressure;
      hoverPressure = basePressure + TAKEOFF_HEIGHT_MM * PRESSURE_CHANGE_TO_ALTITUDE_MM; // Set to hover to 1m above base
      break;
    
    case STARTING:
      // Serial.println("--- Starting ---");
      setSpeed(START_SPEED);
      writeESCs();
      break;
    
    case TAKEOFF:
      // Serial.println("--- Takeoff ---");
      
      // takeOff(averageData.lidar_distance_mm);
      balancePitch(sDataHist);
      balanceRoll(sDataHist);
      balanceAltitudeLidar(sDataHist, TAKEOFF_HEIGHT_MM);
      constrain_hover_pwm();
      writeESCs();
      break;
    
    case HOVERING:
      // Serial.println("--- Hovering ---");
      
      balancePitch(sDataHist);
      balanceRoll(sDataHist);
      // balanceAltitude(sData.pressure.pressure, hoverPressure);
      balanceAltitudeLidar(sDataHist, TAKEOFF_HEIGHT_MM);
      constrain_hover_pwm();
      writeESCs();
      break;
    
    case FLYING:
      // Serial.println("--- Flying ---");
      
      if(xStable(msg)) balancePitch(sDataHist); // Keep balanced if no move command, else move
      else moveX(msg.x_change);
      if(yStable(msg)) balanceRoll(sDataHist);
      else moveY(msg.y_change);
      balanceAltitudeLidar(sDataHist, TAKEOFF_HEIGHT_MM);

      writeESCs();
      break;

    case LANDING:
      // Serial.println("--- Landing Sequence ---");
      digitalWrite(LED_PIN, HIGH);

      forceLand();
      balancePitch(sDataHist);
      balanceRoll(sDataHist);
      // if(averageData.lidar_distance_mm > LANDING_DISTANCE_MM){
      //   balanceAltitudeLidar(sDataHist, LANDING_DISTANCE_MM);
      // } else {
      //   setSpeed(STOP_SPEED);
      // }
      // land();
      break;
    
    case TESTING: // Just read values and print them
      // Serial.println("--- Testing ---");
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      writeESCs(); // write motor speeds

      // delay(500 - MAIN_DELAY_MS);
      break;
  }
}

bool reachedTakeoffHeight(){ // Returns true if drone is at or above takeoff height
  return {abs(averageData.lidar_distance_mm - TAKEOFF_HEIGHT_MM) <= ALTITUDE_THRESHOLD_MM};
}

bool outsideHoveringHeight(){ // Returns true if drone is outside of its min/max height
  return {averageData.lidar_distance_mm < LANDING_DISTANCE_MM || averageData.pressure.pressure > (HEIGHT_LIMIT_HPA + basePressure)};
}

bool landSignal(){ // Returnns true if signal to land has been recieved
  return {averageData.lidar_distance_mm < LANDING_DISTANCE_MM};
}


/* ----- SENSOR READING FUNCTIONS ----- */
void readSensorData(){
  readSensors(sData);
  advanceIndex(sDataHist);
  updateHistory(sDataHist, sData);
}

void sendLogs(const SensorData& data){
  Serial.print("DATA,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(data.accel_x_g, 4);
  Serial.print(",");
  Serial.print(data.accel_y_g, 4);
  Serial.print(",");
  Serial.print(data.accel_z_g, 4);
  Serial.print(",");
  Serial.print(data.gyro_x_dps, 3);
  Serial.print(",");
  Serial.print(data.gyro_y_dps, 3);
  Serial.print(",");
  Serial.print(data.gyro_z_dps, 3);
  Serial.print(",");
  Serial.print(data.lidar_distance_mm);
  Serial.print(",");
  Serial.println(data.pressure.pressure, 4);
}

void sendReadings(const SensorData& data){ // Send readings to Operator
  printSpeeds();
  Serial.print("State: -- "); 
  switch(currentState){
    case OFF: Serial.print("OFF"); break;
    case INIT: Serial.print("INIT"); break;
    case STARTING: Serial.print("STARTING"); break;
    case TAKEOFF: Serial.print("TAKEOFF"); break;
    case HOVERING: Serial.print("HOVERING"); break;
    case FLYING: Serial.print("FLYING"); break;
    case LANDING: Serial.print("LANDING"); break;
    case TESTING: Serial.print("TESTING"); break;
  }
  Serial.println(" --");
  Serial.print("   Acc X (g): "); Serial.print(data.accel_x_g);
  Serial.print(" | Acc Y (g): "); Serial.print(data.accel_y_g);
  Serial.print(" | Acc Z (g): "); Serial.print(data.accel_z_g);
  Serial.print(" | Gyro X (dps): "); Serial.print(data.gyro_x_dps);
  Serial.print(" | Gyro Y (dps): "); Serial.print(data.gyro_y_dps);
  Serial.print(" | Gyro Z (dps): "); Serial.println(data.gyro_z_dps);
  Serial.print("   Dist (mm): "); Serial.print(data.lidar_distance_mm);
  Serial.print(" | DistCorr (mm): "); Serial.print(correctAltitude(data.lidar_distance_mm, data.accel_x_g, data.accel_y_g)); // Correct for tilt
  Serial.print(" | BasPres (hPa): "); Serial.print(basePressure);
  Serial.print(" | Pres (hPa): "); Serial.print(data.pressure.pressure);
  Serial.print(" | Alt (mm): "); Serial.println((data.pressure.pressure - basePressure) / PRESSURE_CHANGE_TO_ALTITUDE_MM);
}

SensorData averageHistory(const SensorDataHistory& history, uint8_t sampleCount) {
  SensorData average{};

  if(sampleCount == 0) {
    return average; // Avoid division by zero
  }
  int32_t lidar_distance_mm_sum = 0;
  for(uint8_t i = 0; i < sampleCount; i++){
    int index = (history.index + NUM_DATA_VALS - i) % NUM_DATA_VALS;

    lidar_distance_mm_sum += history.lidar_distance_mm[index];
    average.pressure.pressure += history.pressure[index].pressure;
  }

  for(uint8_t i = 0; i < sampleCount * IMU_SAMPLES_FACTOR; i++){
    int imu_idx = (history.imu_idx + NUM_DATA_VALS - i) % NUM_DATA_VALS;

    average.accel_x_g += history.accel_x_g[imu_idx];
    average.accel_y_g += history.accel_y_g[imu_idx];
    average.accel_z_g += history.accel_z_g[imu_idx];

    average.gyro_x_dps += history.gyro_x_dps[imu_idx];
    average.gyro_y_dps += history.gyro_y_dps[imu_idx];
    average.gyro_z_dps += history.gyro_z_dps[imu_idx];
  }



  average.accel_x_g /= sampleCount * IMU_SAMPLES_FACTOR;
  average.accel_y_g /= sampleCount * IMU_SAMPLES_FACTOR;
  average.accel_z_g /= sampleCount * IMU_SAMPLES_FACTOR;

  average.gyro_x_dps /= sampleCount * IMU_SAMPLES_FACTOR;
  average.gyro_y_dps /= sampleCount * IMU_SAMPLES_FACTOR;
  average.gyro_z_dps /= sampleCount * IMU_SAMPLES_FACTOR;

  average.lidar_distance_mm = lidar_distance_mm_sum / sampleCount;
  average.pressure.pressure /= sampleCount;
  
  return average;
}


/* ----- MOTOR CONTROL FUNCTIONS ----- */
void land(){ // Landing sequence
  readSensorData();
  static float curPressure = averageData.pressure.pressure;

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

