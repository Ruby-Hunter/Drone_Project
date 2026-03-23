#include <Servo.h>
#include <cmath>
#include <cstring>
#include <Arduino.h>
#include "motor_control.h"
#include "remote_control.h"
#include "sensors.h"
#include "config.h"

SensorData sData; // sensor data
float basePressure, hoverPressure;
float altitude;

// For Sending to Saleae
int16_t pwmx, pwmy, pwmz, pwmdist, pwmpres;

// State Machine states
enum State { OFF, INIT, TAKEOFF, HOVERING, FLYING, LANDING, TESTING };
uint8_t currentState = OFF;
uint32_t lastTick = 0;
int count = 0;

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

// State machine for drone
void stateMachine(){
  if(!digitalRead(DRONE_POWER)){ // Failsafe
    count++;
    if(count > 10){
      if(currentState != OFF && currentState != TESTING){
        currentState = OFF;
        stopMotors();
      }
    }
  } else{
    count = 0;
  }

  /* ============ State Changes ============ */
  switch (currentState){
    case OFF: 
      if(digitalRead(DRONE_POWER)){
        currentState = INIT;
      }
      break;

    case INIT:
      currentState = TAKEOFF;
      break;
    
    case TAKEOFF:
      if(abs(sData.distance_mm - TAKEOFF_HEIGHT_MM) <= ALTITUDE_THRESHOLD_MM){
        currentState = HOVERING;
      }
      break;

    case HOVERING:
      if(sData.distance_mm < LANDING_DISTANCE_MM || sData.pressure.pressure > TAKEOFF_HEIGHT_MM*3*PRESSURE_CHANGE_TO_ALTITUDE_MM + basePressure){ 
        // If drone is close to ground, too high, or pressure sensor is malfunctioning, land
        currentState = LANDING;
      } 
      // else if(!isStable(msg)){
      //   currentState = FLYING;
      // }
      break;
     
    case FLYING:
      if(sData.distance_mm < LANDING_DISTANCE_MM){
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

  /* ============ State Actions ============ */
  switch (currentState){ 
    case OFF:
      Serial.println("Motors off");
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
      break;
    
    case INIT:
      Serial.println("Starting up");
      fiveBlink();
      delay(STARTUP_TIME_MS);  // Wait for ESCs to initialize

      readPressure(sData);
      basePressure = sData.pressure.pressure;
      hoverPressure = basePressure + TAKEOFF_HEIGHT_MM*PRESSURE_CHANGE_TO_ALTITUDE_MM; // Set to hover to 1m above base
      break;
    
    case TAKEOFF:
      Serial.println("Takeoff");
      readValues(sData);
      sendReadings();

      takeOff(sData.distance_mm);
      balancePitch(sData.accel_x_g, sData.gyro_x_dps);
      balanceRoll(sData.accel_y_g, sData.gyro_y_dps);
      writeESCs();
      break;
    
    case HOVERING:
      Serial.println("Hovering");
      readValues(sData);
      sendReadings();

      balancePitch(sData.accel_x_g, sData.gyro_x_dps);
      balanceRoll(sData.accel_y_g, sData.gyro_y_dps);
      // balanceAltitude(sData.pressure.pressure, hoverPressure);
      balanceAltitudeLidar(sData.distance_mm);

      writeESCs();
      break;
    
    case FLYING:
      Serial.println("Flying");
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
      Serial.println("Landing Sequence");
      digitalWrite(LED_PIN, HIGH);
      readValues(sData);
      sendReadings();

      forceLand();
      balancePitch(sData.accel_x_g, sData.gyro_x_dps);
      balanceRoll(sData.accel_y_g, sData.gyro_y_dps);
      // land();
      break;
    
    case TESTING: // Just read values and print them
      Serial.println("Testing Mode");
      digitalWrite(LED_PIN, HIGH);
      stopMotors();

      readValues(sData);
      sendReadings();
      delay(500 - DELAY_MS);
      break;
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

/* ----- SENSOR READING FUNCTIONS ----- */
void sendReadings(){ // Send readings to Saleae
  // Reading on Saleae
  Serial.print("Accel X (g): "); Serial.print(sData.accel_x_g);
  Serial.print(" | Accel Y (g): "); Serial.print(sData.accel_y_g);
  Serial.print(" | Accel Z (g): "); Serial.print(sData.accel_z_g);
  Serial.print(" | Gyro X (dps): "); Serial.print(sData.gyro_x_dps);
  Serial.print(" | Gyro Y (dps): "); Serial.print(sData.gyro_y_dps);
  Serial.print(" | Gyro Z (dps): "); Serial.print(sData.gyro_z_dps);
  Serial.print(" | Distance (mm): "); Serial.print(sData.distance_mm);
  Serial.print(" | BasePressure (hPa): "); Serial.print(basePressure);
  Serial.print(" | Pressure (hPa): "); Serial.print(sData.pressure.pressure);
  Serial.print(" | Altitude (mm): "); Serial.println((sData.pressure.pressure - basePressure) / PRESSURE_CHANGE_TO_ALTITUDE_MM);
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
  static uint16_t pwm = 1200;
  if((getSpeed(1) + getSpeed(2) + getSpeed(3) + getSpeed(4)) > 4800) {
    changeSpeed(-1);
    writeESCs();
  }
  else {
    if(pwm == 1200) Serial.println("--- Hit bottom ---"); // debug
    setSpeed(pwm);
    writeESCs();
    if(pwm > STOP_SPEED){
      pwm -= 5;
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
    delay(200);
    digitalWrite(PC13, HIGH);
    delay(200);
  }
}