#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long stepperStamp;
unsigned long rightStamp;
unsigned long gyroStamp;
unsigned long stepperInterval = 5000;
boolean stepperOn = false;
int leftStepPin = 2;
int leftDirPin = 3;
int rightStepPin = 8;
int rightDirPin = 9;
unsigned long gyroInterval = 5000;

//speed->interval conversion
float speedDeadzone = 0; //motor only starts working at this speed
unsigned long intervalAt100 = 1200;
unsigned long minInterval  = 1500;
unsigned long maxInterval  = 500000;
float exponentiality = 0; //between 0 (linear) and 1 (semi-exponential)

//gyroscope
float previousAngle = 0;
float filterCoeff = 0.99;
//control loop
float targetAngle = -10;
float errorSum = 0;
float Kp = 4;
float Ki = 0.4;
float Kd = 0.1;
//8 0.27 0.03
//6 0.3 0.03
void setup() {
  Serial.begin(9600);
  pinMode(leftStepPin, OUTPUT);
  pinMode(rightStepPin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  Wire.begin();
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  //plotGraph();
}

void loop() {
  checkMicros();
}

void setInterval(float s) {
  if (s < 0) {
    digitalWrite(leftDirPin, LOW);
    digitalWrite(rightDirPin, HIGH);
  }
  if (s > 0) {
    digitalWrite(leftDirPin, HIGH);
    digitalWrite(rightDirPin, LOW);
  }
  s = abs(min(s, 100));
  if (s <= speedDeadzone) stepperOn = false;
  else {
    stepperOn = true;
    stepperInterval =
      constrain(
        (exponentiality * ( //a linear speed-interval formula gives semi-exponential speed control
           map(s, 0.0, 100.0, maxInterval, intervalAt100)
         )) +
        ((1.0 - exponentiality) * ( //a 1/x speed interval formula gives linear speed control
           (intervalAt100 * 100) / s)
        ),
        minInterval, maxInterval) / 2;
  }
}

void checkMicros() {
  unsigned long currentMicros = micros();
  if (stepperOn and currentMicros > stepperStamp + stepperInterval) {
    digitalWrite(leftStepPin, HIGH);
    digitalWrite(leftStepPin, LOW);
    digitalWrite(rightStepPin, HIGH);
    digitalWrite(rightStepPin, LOW);
    stepperStamp = stepperStamp + stepperInterval;
  }
  if (currentMicros > gyroStamp + gyroInterval) {
    unsigned long loopTime = (currentMicros - gyroStamp) / 1000;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float accAngle =  atan2(az, ax) * RAD_TO_DEG;
    int gyroRate = map(gy - 10, -32768, 32768, -250, 250);
    float gyroAngle = (float)gyroRate * loopTime / 1000;
    float currentAngle = filterCoeff * (previousAngle + gyroAngle) + ( (1 - filterCoeff) * accAngle);
    float error = currentAngle - targetAngle;
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -300, 300);
    float motorPower = Kp * (error) + Ki * (errorSum) * loopTime - Kd * (currentAngle - previousAngle) / loopTime;
    setInterval(motorPower);
    previousAngle = currentAngle;
    gyroStamp = currentMicros;
    //if (abs(motorPower) < 3) {
    //  targetAngle = (0.99 * targetAngle) + (0.01 * currentAngle);
   // }
   //Serial.println(currentAngle);
  }
}

void plotGraph() {
  for (int x = 0; x <= 100; x++) {
    setInterval(x);
    Serial.println(stepperInterval);
    delay(10);
  }
}


