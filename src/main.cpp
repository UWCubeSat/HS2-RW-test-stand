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
#define PWM_DEADZONE 0.132 // needs to be at least this percent of PWM_BOUNDS=200 to do have motor spin
#define PWM_STATIONARY 1500
#define REVERSED 0
Servo ESC;
double motorSpeed = 0;
// ------------------

// ------BNO055----- (GYRO)
// DOC STRINGS: https://www.arduino.cc/reference/en/libraries/adafruit-bno055/ https://github.com/adafruit/Adafruit_BNO055 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #define MPU_ADDRESS 0x28 // I2C address of the BNO055
#define BNO055_SAMPLERATE_DELAY_MS 10
Adafruit_BNO055 bno = Adafruit_BNO055(55);
double yawAngle = 0;
double yawAngularSpeed = 0;

double gyroHeadingAngle = 0;
double gyroPitchAngle = 0;
uint8_t* gyroReserved = 0; 
double gyroRoll= 0; 
double gyroStatus = 0; 

float* gyroV = 0; 
double gyroX = 0; 
double gyroY = 0; 
double gyroZ = 0; 

// ------------------

// -------PID--------
#include "PID.h"
const double xkP = 0.05;  
const double xkI = 0.0;
const double xkD = 400; 
PIDAngleController pidAngle(xkP, xkI, xkD); 
const float MOE = 1; // degrees
// const double vkP = 0.00006; // works best for detumble
const double vkP = 0.0013; // works best for movement. Cursed. Also switching between constants can be done with public functions
const double vkD = 0.0013;
// const double vkP = 0.007; 
// const double vkD = 0.00; 
const double vkI = 0.0;
PIDController pidSpeed(vkP, vkI, vkD);
// ------------------

// -------SD---------
#include "SD.h"
const int chip_select = 10;
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

  gyroPitchAngle = event.gyro.pitch;
  gyroHeadingAngle = event.gyro.heading;

  gyroReserved = event.gyro.reserved;
  gyroRoll = event.gyro.roll;
  gyroStatus = event.gyro.status;
  gyroV = event.gyro.v;
  gyroX = event.gyro.x;
  gyroY = event.gyro.y;
  gyroZ = event.gyro.z;

}

void updateYawSpeed() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  yawAngularSpeed = gyro.z(); // degrees/sec
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
    Serial.print(" pwm: "); Serial.print(motorSpeed);
  #endif
  ESC.writeMicroseconds(motorSpeed);
}

// Set the current speed from -1 to 1 as a percentage of speed
void setPercentSpeed (double percent) {
  if (fabs(percent) < PWM_DEADZONE) percent = 0;
  percent = constrain(percent, -1.0, 1.0);
  setSpeed(percent * PWM_BOUNDS);
}

void printMoreThings() {
    sensors_event_t event;
    bno.getEvent(&event);
    // Serial.print(" roll: "); Serial.print(event.orientation.roll);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Serial.print("\tgyro z:\t"); Serial.print(gyro.z());
    Serial.println(F(" "));
}

void printDirectory(File dir, int num_tabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (!entry) {
      break;
    }
    for (uint8_t i = 0; i < num_tabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, num_tabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
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

  pinMode(SS, OUTPUT);
  if (!SD.begin(chip_select)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  // if (SD.exists("test.txt")) {
  //   SD.remove("test.txt");
  // }

  File root = SD.open("/");
  printDirectory(root, 0);

  // file = SD.open("test.txt", FILE_WRITE);
  // if (file) {
  //   file.println("hi");
  //   file.close();
  // } else {
  //   Serial.println("card failed write");
  // }
  // file = SD.open("test.txt", FILE_READ);
  // if (file) {
  //   Serial.println("test.txt:");

  //   // read from the file until there's nothing else in it:
  //   while (file.available()) {
  //     Serial.write(file.read());
  //   }
  //   // close the file:
  //   file.close();
  // } else {
  //   Serial.println("card failed read");
  // }

  Serial.println("Setup finished!");

  timeCur = millis();
  timeStart = timeCur;
}

int countLoops = 0;

void loop() {
  // if (countLoops < 30) {
  //   motorSpeed = max(30, motorSpeed);
  //   countLoops++;
  // } 
  // Every 10ms, read IMU and call controllers
  if (millis() - timeCur > 10) {
    timePrev = timeCur;
    timeCur = millis();

    // update values
    updateYawAngle();
    updateYawSpeed();
    updateRollingAvg();

    // **** TESTING BNO055 ATTRIBUTES ***** //

    Serial.print("Heading Angle: ");
    Serial.println(gyroHeadingAngle);

    Serial.print("Pitch Angle: ");
    Serial.println(gyroPitchAngle);

    Serial.print("Gyro Reserved: ");
    Serial.println(reinterpret_cast<uintptr_t>(gyroReserved));

    Serial.print("Gyro Roll: ");
    Serial.println(gyroRoll);

    Serial.print("Gyro Status: ");
    Serial.println(gyroStatus);

    Serial.print("Gyro V: ");
    Serial.println(reinterpret_cast<uintptr_t>(gyroV));

    Serial.print("Gyro X: ");
    Serial.println(gyroX);

    Serial.print("Gyro Y: ");
    Serial.println(gyroY);

    Serial.print("Gyro Z: ");
    Serial.println(gyroZ);






    // // FSM transition
    // if (controllerState == 1 && fabs(rollingAvg) > 360 /* °/s */) { // this should be the saturated deg/sec
    //   controllerState = 0;
    // } else if (controllerState == 0 && fabs(rollingAvg) < 45 /* °/s */) {
    //   controllerState = 1;
    // }
    // controllerState = 1;
    
    // // FSM action 
    // float update = 0;
    // if (controllerState == 0) { // state 0 = detumble + update time for angle 
    //   // update = pidSpeed.compute(0, rollingAvg, timeCur - timePrev); 
    //   update = pidSpeed.compute(0, rollingAvg, timeCur - timePrev); 
    //   pidAngle.compute(targetPos, yawAngle, timeCur - timePrev); // need because time is updated each iter
    // } else { // state 1 = set setpoint to calculated motor output. Error = desired pwm -avg pwm 
    //   float anglePwmOut = pidAngle.compute(targetPos, yawAngle, timeCur - timePrev);
    //   update = pidSpeed.compute(anglePwmOut, rollingAvg, timeCur - timePrev);
    //   #if TEST
    //     Serial.print("anglePwmOut: "); Serial.print(anglePwmOut);
    //   #endif
    // } 
    // if (fabs(pidAngle.getError()) <= MOE) update = 0; // if withing MOE, keep constant velocity
    // if (fabs(pidSpeed.getError()) <= MOE) update = 0; // if withing MOE, keep constant velocity
    // motorSpeed += update;
    // // There's a deadzone where motor speed doesn't do anything but it still takes time to cross ~= +/- 20
    // if (motorSpeed < 20 && motorSpeed > -19) motorSpeed = -20;
    // else if (motorSpeed > -20 && motorSpeed < 19) motorSpeed = 20;

    // motorSpeed = constrain(motorSpeed, -PWM_BOUNDS, PWM_BOUNDS);
    #if TEST
      // Serial.print(" time step: "); Serial.println(timeCur - timePrev);
      // Serial.print(" rollingAvg: "); Serial.print(rollingAvg);
      Serial.print(" posError: "); Serial.print(pidAngle.getError());
      Serial.print(" speedError: "); Serial.print(pidSpeed.getError());
      Serial.print(" update: "); Serial.print(update);
      Serial.print(" motorSpeed: "); Serial.print(motorSpeed);
    #endif

    // motorSpeed = pidAngle.compute(targetPos, yawAngle, timeCur - timePrev); // need because time is updated each iter
    // setSpeed(motorSpeed);

    // printMoreThings();
    #if TEST
      Serial.println(" ");
    #endif

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}
