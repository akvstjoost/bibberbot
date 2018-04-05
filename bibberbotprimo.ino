#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <WiFiLink.h>
#include <PubSubClient.h>
#include "secret.h"
#include <stdlib.h>
#define LEFTSTEPPER 0
#define RIGHTSTEPPER 1
#define USE_WIFI 1

//stepper
unsigned long stepperStamp[] = {0, 0};
unsigned long stepperInterval[] = {5000, 5000};
boolean stepperOn[] = {false, false};
int stepperStepPin[] = {2, 4};
int stepperDirPin[] = {3, 5};


//speed->interval conversion
float speedDeadzone = 0; //motor only starts working at this speed
unsigned long intervalAt100 = 1200;
unsigned long minInterval  = 1500;
unsigned long maxInterval  = 500000;

//gyroscope
MPU6050 accelgyro;
unsigned long gyroStamp;
unsigned long gyroInterval = 5000;
float previousAngle = 0;
float filterCoeff = 0.99;
//control loop
float balanceAngle = -10;
float errorSum = 0;
float Kp = 4;
float Ki = 0.4;
float Kd = 0.1;
//8 0.27 0.03
//6 0.3 0.03

//controller
int controllerSpeed = 0;
int controllerSteer = 0;

//mqtt
const char* mqtt_server = "broker.mqtt-dashboard.com";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long lastReconnectAttempt = 0;

void setup() {
  Serial.begin(9600);
  pinMode(stepperStepPin[LEFTSTEPPER], OUTPUT);
  pinMode(stepperStepPin[RIGHTSTEPPER], OUTPUT);
  pinMode(stepperDirPin[LEFTSTEPPER], OUTPUT);
  pinMode(stepperDirPin[RIGHTSTEPPER], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Wire.begin();
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  if (USE_WIFI) {
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  }
}

void loop() {
  balanceloop();
  if (USE_WIFI) mqttloop();
}

void setInterval(float s, int stepper) { // stepper: 0 for left stepper, 1 for right stepper
  digitalWrite(stepperDirPin[stepper], (s < 0) ? stepper : !stepper); //speed polarity determines direction, but both sides have inverse directions;
  s = abs(min(s, 100));
  if (s <= speedDeadzone) stepperOn[stepper] = false;
  else {
    stepperOn[stepper] = true;
    stepperInterval[stepper] =
      constrain((intervalAt100 * 100) / s, minInterval, maxInterval) / 2;
  }
}

void balanceloop() {
  unsigned long currentMicros = micros();
  for (int i = LEFTSTEPPER; i <= RIGHTSTEPPER; i++) {
    if (stepperOn[i] and currentMicros > stepperStamp[i] + stepperInterval[i]) {
      digitalWrite(stepperStepPin[i], HIGH);
      digitalWrite(stepperStepPin[i], LOW);
      stepperStamp[i] = stepperStamp[i] + stepperInterval[i];
    }

  }
  if (currentMicros > gyroStamp + gyroInterval) {
    float targetAngle = balanceAngle + (controllerSpeed / 10);
    int16_t ax, ay, az, gx, gy, gz;
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
    setInterval(motorPower + controllerSteer, LEFTSTEPPER);
    setInterval(motorPower - controllerSteer, RIGHTSTEPPER);
    previousAngle = currentAngle;
    gyroStamp = currentMicros;
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

void callback(char* topic, byte* payload, unsigned int length) {
  char val[length + 1];
  for (int i = 0; i < length; i++) {
    val[i] = (char)payload[i];
  }
  val[length + 1] = '\0';
  if (!strcmp(topic, "bibberBot/speed")) controllerSpeed = atoi(val);
  if (!strcmp(topic, "bibberBot/steer")) controllerSteer = atoi(val);
}


boolean reconnect() {
  if (client.connect("bibberBot")) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("connected");
    client.subscribe("bibberBot/speed");
    client.subscribe("bibberBot/steer");
  }
  return client.connected();
}

void mqttloop() {
  long now = millis();
  if (!client.connected()) {
    if (now - lastReconnectAttempt > 5000) {
      digitalWrite(LED_BUILTIN, LOW);
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {

    client.loop();
    /*
      if (now - lastMsg > 2000) {
      lastMsg = now;

      client.publish("outTopic", "hello world");
      }
    */
  }

}
