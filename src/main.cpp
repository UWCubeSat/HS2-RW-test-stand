// ------GENERAL-----
#define TEST 0
long timeCur, timePrev, timeStart; 
const int numReadings = 5;
double readings[numReadings];
int readIndex = 0;
double total = 0;
double rollingAvg = 0;
double targetPos = 0;

// ------BLDC--------
#include <Servo.h>
#define PWM_BOUNDS 200 // range is PWM_STATIONARY +/- PWM_BOUNDS
#define PWM_STATIONARY 1500
#define REVERSED 1
Servo ESC;
double motorSpeed = 0;
// ------------------

// ------BNO055-----
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #define MPU_ADDRESS 0x28 // I2C address of the BNO055
#define BNO055_SAMPLERATE_DELAY_MS 10
Adafruit_BNO055 bno = Adafruit_BNO055(55);
double yawAngle = 0;
double yawAngularSpeed = 0;
// ------------------

// -------PID--------
#include "PID.h"
const double xkP = 1; // displacement kP 
const double xkI = 0.0;
const double xkD = 400; 
PIDAngleController pidAngle(xkP, xkI, xkD); 
const float MOE = 0.05; // margin of error is 0.05 degrees - I cry
// TODO: ESC already has a velocity speed, so sus. Consider 
const double vkP = 0.05; // velocity kP 
const double vkI = 0.000;
const double vkD = 0.017; 
PIDController pidSpeed(vkP, vkI, vkD);
// ------------------

// FSM variables
byte controllerState = 0;
// ------------------

// Calibrate ESC by plugging in battery when MAX_PWM is being outputted.
void calibrateESC() {
  ESC.writeMicroseconds(PWM_STATIONARY);
  delay(2000);
}

void updateYawAngle() {
  sensors_event_t event;
  bno.getEvent(&event);
  yawAngle = event.orientation.roll;
}

void updateYawSpeed() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  yawAngularSpeed = gyro.x(); // degrees/sec
}

// Smooth the angular speed --> rolling average
void updateRollingAvg() {
    total = total - readings[readIndex]; 
    readings[readIndex] = yawAngularSpeed; 
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) { readIndex = 0; }
    rollingAvg = total / numReadings;
}

// Set the current speed and direction of the motor
void setSpeed(double targetSpeed) {
  #if REVERSED
    targetSpeed *= -1.0;
  #endif

  float motorSpeed = PWM_STATIONARY + targetSpeed;
  #if TEST
    Serial.print(" pwm: "); Serial.println(motorSpeed);
  #endif
  ESC.writeMicroseconds(motorSpeed);
}

// Set the current speed from -1 to 1 as a percentage of speed
void setPercentSpeed (double percent) {
  percent = constrain(percent, -1.0, 1.0);
  setSpeed(percent * PWM_BOUNDS);
}

void printMoreThings() {
    // Print info to console
    // Serial.print("motorSpeed: "); Serial.print(motorSpeed);
    // Serial.print(" time step: "); Serial.println(timeCur - timePrev);

    sensors_event_t event;
    bno.getEvent(&event);
    // Serial.print(" roll: "); Serial.print(event.orientation.roll);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Serial.print("\tgyro z:\t"); Serial.print(gyro.z());
    Serial.println(F(" "));
}

void setup() {
  Serial.begin(9600);
  
  // IMU setup
  if(!bno.begin()) {
    Serial.print(F("No BNO055 detected"));
    exit(1);
  }
  bno.setExtCrystalUse(true);
  
  // Start ESC on pin 9
  ESC.attach(9,1000,2000);
  calibrateESC();

  updateYawAngle();
  targetPos = yawAngle;

  Serial.println("Setup finished!");

  timeCur = millis();
  timeStart = timeCur;
}

void loop() {
  // Every 10ms, read IMU and call controllers
  if (millis() - timeCur > 10) {
    timePrev = timeCur;
    timeCur = millis();

    // update values
    updateYawAngle();
    updateYawSpeed();
    updateRollingAvg();

    setPercentSpeed(.2);

    // // FSM transition
    // if (controllerState == 1 && fabs(rollingAvg) > 360 /* °/s */) {
    //   controllerState = 0;
    // } else if (controllerState == 0 && fabs(rollingAvg) < 45 /* °/s */) {
    //   controllerState = 1;
    // }
    // float update = 0;
    // // FSM action 
    // if (controllerState == 0) { // state 0 = detumble + update time for angle 
    //   update = pidSpeed.compute(0, rollingAvg, timeCur - timePrev);
    //   pidAngle.compute(targetPos, yawAngle, timeCur - timePrev); // need because time is updated each iter
    // } else { // state 1 = set setpoint to calculated motor output. Error = desired pwm -avg pwm 
    //   float anglePwmOut = pidAngle.compute(targetPos, yawAngle, timeCur - timePrev);
    //   #if TEST
    //     Serial.print("anglePwmOut: "); Serial.print(anglePwmOut);
    //     Serial.print(" rollingAvg: "); Serial.print(rollingAvg);
    //   #endif
    //   update = pidSpeed.compute(anglePwmOut, rollingAvg, timeCur - timePrev);
    // } 
    // motorSpeed += update;
    // motorSpeed = constrain(motorSpeed, -PWM_BOUNDS, PWM_BOUNDS);
    // if (fabs(pidAngle.getError()) <= MOE) motorSpeed = 0;
    // #if TEST
    //   Serial.print(" update: "); Serial.print(update);
    //   Serial.print(" motorSpeed: "); Serial.print(motorSpeed);
    // #endif
    // printMoreThings();

    // // motorSpeed = pidAngle.compute(targetPos, yawAngle, timeCur - timePrev); // need because time is updated each iter

    // setSpeed(motorSpeed);

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}
