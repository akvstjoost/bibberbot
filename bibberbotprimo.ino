#include <WiFiLink.h>
#include <PubSubClient.h>
#include "secret.h"
#include <stdlib.h>
#define LEFTSTEPPER 0
#define RIGHTSTEPPER 1
#define USE_WIFI 1

//stepper
unsigned long stepperStamp[] = {0, 0};
unsigned long stepperInterval[] = {5000, 5000}; //uS
boolean stepperOn[] = {false, false};
int stepperStepPin[] = {2, 4};
int stepperDirPin[] = {3, 5};


//speed->interval conversion
float speedDeadzone = 0; //motor only starts working at this speed
unsigned long intervalAt100 = 1200;
unsigned long minInterval  = 1500;
unsigned long maxInterval  = 500000;

//gyroscope
unsigned long gyroStamp = 0;
unsigned long gyroInterval = 5000; //uS
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
unsigned long mqttStamp = 0;
unsigned long mqttInterval = 100000; //uS
unsigned long lastMsg = 0;
unsigned long lastReconnectAttempt = 0;


void setup() {
  Serial.begin(9600);
  pinMode(stepperStepPin[LEFTSTEPPER], OUTPUT);
  pinMode(stepperStepPin[RIGHTSTEPPER], OUTPUT);
  pinMode(stepperDirPin[LEFTSTEPPER], OUTPUT);
  pinMode(stepperDirPin[RIGHTSTEPPER], OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  setupMPU6050();
  if (USE_WIFI) {
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  }
}

void loop() {
  microsloop();
  Serial.println(controllerSteer);
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

void microsloop() {
  unsigned long currentMicros = micros();
  if (currentMicros < gyroStamp) { //fix overflow
    gyroStamp = 0;
    stepperStamp[0] = 0;
    stepperStamp[1] = 0;
    mqttStamp = 0;
  }
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
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
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
      //client.publish("outTopic", "hello world");
      mqttStamp = currentMicros;
    }
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


