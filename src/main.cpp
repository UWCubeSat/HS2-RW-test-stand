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
const double xkP = 2.5; // displacement kP 
const double xkI = 0.0;
const double xkD = 400; 
PIDAngleController pidAngle(xkP, xkI, xkD); 
const float MOE = 0.05; // margin of error is 0.05 degrees - I cry
const double vkP = 0.05; // velocity kP // TODO: ESC already has a velocity speed, so sus.
const double vkI = 0.000;
const double vkD = 0.017; 
PIDController pidSpeed(vkP, vkI, vkD);
// ------------------

// ------GENERAL-----
long timeCur, timePrev, timeStart; 
const int numReadings = 5;
double readings[numReadings];
int readIndex = 0;
double total = 0;
double rollingAvg = 0;
double targetPos = 0;
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
  sensors_event_t event;
  bno.getEvent(&event);
  yawAngularSpeed = event.gyro.x;
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
  Serial.print(" pwm: "); Serial.println(motorSpeed);
  ESC.writeMicroseconds(motorSpeed);
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

    // FSM transition
    if (controllerState == 1 && fabs(rollingAvg) > 360 /* °/s */) {
      controllerState = 0;
    } else if (controllerState == 0 && fabs(rollingAvg) < 45 /* °/s */) {
      controllerState = 1;
    }
    float update = 0;
    // FSM action 
    if (controllerState == 0) { // state 0 = detumble + update time for angle 
      update = pidSpeed.compute(0, rollingAvg, timeCur - timePrev);
      pidAngle.compute(targetPos, yawAngle, timeCur - timePrev); // need because time is updated each iter
    } else { // state 1 = set setpoint to calculated motor output. Error = desired pwm -avg pwm 
      float anglePwmOut = pidAngle.compute(targetPos, yawAngle, timeCur - timePrev);
      Serial.print("anglePwmOut: "); Serial.print(anglePwmOut);
      update = pidSpeed.compute(anglePwmOut, rollingAvg, timeCur - timePrev);
    } 
    motorSpeed += update;
    motorSpeed = constrain(motorSpeed, -PWM_BOUNDS, PWM_BOUNDS);
    if (fabs(pidAngle.getError()) <= MOE) motorSpeed = 0;
    Serial.print(" update: "); Serial.print(update);
    Serial.print(" motorSpeed: "); Serial.print(motorSpeed);

    // motorSpeed = pidAngle.compute(targetPos, yawAngle, timeCur - timePrev); // need because time is updated each iter

    setSpeed(motorSpeed);

    delay(BNO055_SAMPLERATE_DELAY_MS);

    // Print info to console
    // Serial.print("motorSpeed: "); Serial.print(motorSpeed);
    // goes from -5 to 355. BAD
    // Serial.print(" time step: "); Serial.println(timeCur - timePrev);
    // time step is ~35 with just these 2, ~115 with all prints

    // sensors_event_t event;
    // bno.getEvent(&event);
    // Serial.print(" roll: "); Serial.print(event.orientation.roll);
    // Serial.print(" heading: "); Serial.print(event.orientation.heading);
    // Serial.print(" gyro x: "); Serial.print(event.gyro.x);
    // Serial.print(" gyro y: "); Serial.print(event.gyro.y);
    // Serial.print(" gyro z: "); Serial.println(event.gyro.z);
    // Serial.print(F(" "));
    // Serial.println(rollingAvg);
  }
}
