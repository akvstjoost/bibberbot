#include <PubSubClient.h>
#include <WiFiLink.h>
#include "secret.h"
#include <stdlib.h>

#define LEFTSTEPPER 0
#define RIGHTSTEPPER 1
+
#define USE_WIFI 1
#define USE_STEPPERS 1

#define PRINT_GYRO 0
#define PRINT_BATTERY 0

//stepper
unsigned long stepperStamp[] = {0, 0};
unsigned long stepperInterval[] = {10000, 10000}; //uS
boolean stepperOn[] = {false, false};
int stepperStepPin[] = {A5, A3};
int stepperDirPin[] = {A4, A2};
int stepperMicroPin[] = {7, 6, 5};
byte stepDivider = B010; //2s

//speed->interval conversion
float speedDeadzone = 0; //motor only starts working at this speed
unsigned long intervalAt100 = 1200;
unsigned long minInterval  = 1400;
unsigned long maxInterval  = 500000;
float quadraticness = 0;

//gyroscope
unsigned long gyroStamp = 0;
unsigned long gyroInterval = 5000; //uS
float previousAngle = 0;
float filterCoeff = 0.995;
//control loop
float balanceAngle = -1;
float errorSum = 0;
float Kp = 5;
float Ki = 0.4;
float Kd = 0.05;
int maxError = 100;
//4 0.4 0.1

//controller
int controllerSpeed = 0;
int controllerSteer = 0;

//battery
unsigned long batteryStamp = 0;
unsigned long batteryInterval = 1000000;
float batteryVoltage = 99;

//mqtt
const char* mqtt_server = "broker.mqtt-dashboard.com";
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long mqttStamp = 0;
unsigned long mqttInterval = 100000; //uS
unsigned long lastReconnectAttempt = 0;


void setup() {
  Serial.begin(9600);
  pinMode(stepperStepPin[LEFTSTEPPER], OUTPUT);
  pinMode(stepperStepPin[RIGHTSTEPPER], OUTPUT);
  pinMode(stepperDirPin[LEFTSTEPPER], OUTPUT);
  pinMode(stepperDirPin[RIGHTSTEPPER], OUTPUT);
  pinMode(stepperMicroPin[0], OUTPUT);
  pinMode(stepperMicroPin[1], OUTPUT);
  pinMode(stepperMicroPin[2], OUTPUT);
  digitalWrite(stepperMicroPin[0], (stepDivider & 0x4));
  digitalWrite(stepperMicroPin[1], (stepDivider & 0x2));
  digitalWrite(stepperMicroPin[2], (stepDivider & 0x1));
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  setupMPU6050();
  if (USE_WIFI) {
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  }
  delay(1000);

}

void loop() {
  microsloop();
}

void setInterval(float s, int stepper) { // stepper: 0 for left stepper, 1 for right stepper
  digitalWrite(stepperDirPin[stepper], (s < 0) ? stepper : !stepper); //speed polarity determines direction, but both sides have inverse directions;
  s = abs(min(s * (1 + (s * quadraticness)), 100)); //quadraticness
  if (s <= speedDeadzone) stepperOn[stepper] = false;
  else {
    stepperOn[stepper] = true;
    stepperInterval[stepper] =
      constrain((intervalAt100 * 100) / s, minInterval, maxInterval) / 4;
  }
}

void microsloop() {
  unsigned long currentMicros = micros();

  if (currentMicros < gyroStamp) { //fix overflow
    gyroStamp = 0;
    stepperStamp[0] = 0;
    stepperStamp[1] = 0;
    mqttStamp = 0;
  }
  if (USE_STEPPERS) {

    for (int i = LEFTSTEPPER; i <= RIGHTSTEPPER; i++) {
      if (stepperOn[i] and currentMicros > stepperStamp[i] + stepperInterval[i]) {
        digitalWrite(stepperStepPin[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(stepperStepPin[i], LOW);
        stepperStamp[i] = max(stepperStamp[i] + stepperInterval[i], currentMicros);
      }
    }

  }
  if (currentMicros > gyroStamp + gyroInterval) {
    float targetAngle = balanceAngle + (controllerSpeed / 10);
    int16_t ax, ay, az, gx, gy, gz;
    unsigned long loopTime = (currentMicros - gyroStamp) / 1000;
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float accAngle =  (atan2(ay, az) * RAD_TO_DEG) - 90;
    int gyroRate = map(gx - 1370, -32768, 32768, -250, 250);
    float gyroAngle = (float)gyroRate * loopTime / 1000;
    float currentAngle = filterCoeff * (previousAngle + gyroAngle) + ( (1 - filterCoeff) * accAngle);
    float error = currentAngle - targetAngle;
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -maxError, maxError);
    float motorPower = (Kp * (error)) + (Ki * (errorSum) * loopTime) - ( Kd * (currentAngle - previousAngle) / loopTime);

    setInterval(motorPower + controllerSteer, LEFTSTEPPER);
    setInterval(motorPower - controllerSteer, RIGHTSTEPPER);
    previousAngle = currentAngle;
    gyroStamp = currentMicros;
    if (PRINT_GYRO) {
      Serial.print(accAngle);
      Serial.print("\t");
      Serial.print(gyroAngle);
      Serial.print("\t");
      Serial.print(currentAngle);
      Serial.print("\t");
      Serial.print(stepperInterval[0]);
      Serial.print("\t");
      Serial.print(stepperInterval[1]);
      Serial.print("\t");
      Serial.print(controllerSteer);
      Serial.print("\t");
      Serial.println(controllerSpeed);
    }
  }
  if (USE_WIFI && (currentMicros > mqttStamp + mqttInterval)) {
    if (!client.connected()) {
      if (currentMicros > lastReconnectAttempt + 5000000) {
        digitalWrite(LED_BUILTIN, LOW);
        lastReconnectAttempt = currentMicros;
        // Attempt to reconnect
        if (reconnect()) {
          lastReconnectAttempt = 0;
        }
      }
    } else {

      client.loop();
      mqttStamp = currentMicros;
    }
  }
  if (currentMicros > batteryStamp + batteryInterval) {
    batteryVoltage = analogRead(A0) * 3.3 / 1024.0;
    if (PRINT_BATTERY) {
      Serial.print("Battery V:");
      Serial.print(batteryVoltage);
    }
    batteryStamp = currentMicros;
  }

}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(NETWORK);
  WiFi.begin(NETWORK, PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte * payload, unsigned int length) { //takes about 10 ms
  unsigned long m = millis();
  char val[length + 1];
  for (int i = 0; i < length; i++) {
    val[i] = (char)payload[i];
  }
  val[length + 1] = '\0';
  if (!strcmp(topic, "bibberBot/speed")) controllerSpeed = atoi(val);
  else if (!strcmp(topic, "bibberBot/steer")) controllerSteer = atoi(val);
  else if (!strcmp(topic, "bibberBot/kp")) Kp = atof(val);
  else if (!strcmp(topic, "bibberBot/ki")) Ki = atof(val);
  else if (!strcmp(topic, "bibberBot/kd")) Kd = atof(val);
  else if (!strcmp(topic, "bibberBot/balance")) balanceAngle = atof(val);
  else if (!strcmp(topic, "bibberBot/deadzone")) speedDeadzone = atof(val);
  else if (!strcmp(topic, "bibberBot/quadraticness")) quadraticness = atof(val);
  else if (!strcmp(topic, "bibberBot/maxerror")) quadraticness = atoi(val);

}


boolean reconnect() {
  if (client.connect("bibberBot")) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("connected");
    client.subscribe("bibberBot/speed");
    client.subscribe("bibberBot/steer");
    client.subscribe("bibberBot/kp");
    client.subscribe("bibberBot/ki");
    client.subscribe("bibberBot/kd");
    client.subscribe("bibberBot/balance");
    client.subscribe("bibberBot/deadzone");
    client.subscribe("bibberBot/quadraticness");
    client.subscribe("bibberBot/maxerror");
  }
  return client.connected();

}
