#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

unsigned long stepperStamp;
unsigned long rightStamp;
unsigned long gyroStamp;
unsigned long stepperInterval = 5000; //means off
boolean stepperOn = false;
int leftStepPin = 2;
int leftDirPin = 3;
int rightStepPin = 8;
int rightDirPin = 9;
unsigned long gyroInterval = 5000;

//speed->interval conversion
float speedDeadzone = 0.5; //motor only starts working at this speed
unsigned long intervalAt100 = 1000;
unsigned long minInterval  = 1000;
unsigned long maxInterval  = 10000;
float exponentiality = 0; //between 0 (linear) and 1 (semi-exponential)

//gyroscope
float previousAngle = 0;
float filterCoeff = 0.996;
//control loop
float targetAngle = -1.5;
float errorSum = 0;
float Kp = 5;
float Ki = 0.1;
float Kd = 0.2;
//3.3,0.07,0.02valmost
//4.2 0.09 0.02
float lastls = 0;
float lastrs = 0;

void setup() {
  Serial.begin(9600);
  pinMode(leftStepPin, OUTPUT);
  pinMode(rightStepPin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  Wire.begin();
  if (!imu.init()) Serial.println("Failed to detect and initialize IMU!");
  imu.enableDefault();
  //plotGraph();
}

void loop() {
  checkMicros();
}

void setInterval(float s) {
  if (s > 0) digitalWrite(leftDirPin, LOW);
  if (s < 0) digitalWrite(leftDirPin, HIGH);
  s = abs(min(s,100));
  if (s <= speedDeadzone) stepperOn = false;
  else {
    stepperOn = true;
    stepperInterval =
      constrain(
        (exponentiality * ( //a linear speed-interval formula gives semi-exponential speed control
           map(s, 0.0, 100.0, maxInterval, intervalAt100)
         )) +
        ((1.0 - exponentiality) * ( //a 1/x speed interval formula gives linear speed control
           (intervalAt100*100) / s)
        ),
      minInterval, maxInterval);
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
    imu.read();
    float accAngle = 0 - atan2(imu.a.z, 0 - imu.a.x) * RAD_TO_DEG;
    int gyroX = imu.g.y;
    int gyroRate = map(gyroX + 897, -32768, 32767, -290, 290);
    float gyroAngle = (float)gyroRate * loopTime / 1000;
    float currentAngle = filterCoeff * (previousAngle + gyroAngle) + ( (1 - filterCoeff) * accAngle);
    float error = currentAngle - targetAngle;
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -300, 300);
    float motorPower = Kp * (error) + Ki * (errorSum) * loopTime - Kd * (currentAngle - previousAngle) / loopTime;
    setInterval(motorPower);
    previousAngle = currentAngle;
    gyroStamp = currentMicros;
  }
}

void plotGraph() {
  for (int x=0;x<=100;x++) {
  setInterval(x);
  Serial.println(stepperInterval);
  delay(10);
  }
}

