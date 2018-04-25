#include <WiFiLink.h>
#include <PubSubClient.h>
#include "secret.h"
#include <stdlib.h>

#define LEFTSTEPPER 0
#define RIGHTSTEPPER 1
#define USE_WIFI 0
#define USE_STEPPERS 1
#define PRINT_GYRO 0

//stepper
unsigned long stepperStamp[] = {0, 0};
unsigned long stepperInterval[] = {10000, 10000}; //uS
boolean stepperOn[] = {false,false};
int stepperStepPin[] = {10, 12};
int stepperDirPin[] = {11, 13};
int stepperMicroPin[] = {7, 5, 6};
byte stepDivider = B001; //quarters

//speed->interval conversion
float speedDeadzone = 0; //motor only starts working at this speed
unsigned long intervalAt100 = 2400;
unsigned long minInterval  = 3000;
unsigned long maxInterval  = 500000;

//gyroscope
unsigned long gyroStamp = 0;
unsigned long gyroInterval = 5000; //uS
float previousAngle = 0;
float filterCoeff = 0.99;
//control loop
float balanceAngle = 0;
float errorSum = 0;
float Kp = 2;
float Ki = 0.5;
float Kd = 0.1;
//4 0.4 0.1

//controller
int controllerSpeed = 0;
int controllerSteer = 0;

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

}

void loop() {
  microsloop();
}

void setInterval(float s, int stepper) { // stepper: 0 for left stepper, 1 for right stepper
  digitalWrite(stepperDirPin[stepper], (s < 0) ? stepper : !stepper); //speed polarity determines direction, but both sides have inverse directions;
  s = abs(min(s, 100));
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
    errorSum = constrain(errorSum, -300, 300);
    float motorPower = Kp * (error) + Ki * (errorSum) * loopTime - Kd * (currentAngle - previousAngle) / loopTime;
    setInterval(motorPower + controllerSteer, LEFTSTEPPER);
    setInterval(motorPower - controllerSteer, RIGHTSTEPPER);
    previousAngle = currentAngle;
    gyroStamp = currentMicros;
    if (PRINT_GYRO) { 
      Serial.print(accAngle);
      Serial.print("\t");
      Serial.print(gyroAngle);
      Serial.print("\t");
      Serial.println(currentAngle);
      delay(100);
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
      //client.publish("bibberBot/out", "hello world");
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
  unsigned long m = millis();
  char val[length + 1];
  for (int i = 0; i < length; i++) {
    val[i] = (char)payload[i];
  }
  val[length + 1] = '\0';
  if (!strcmp(topic, "bibberBot/speed")) controllerSpeed = atoi(val);
  if (!strcmp(topic, "bibberBot/steer")) controllerSteer = atoi(val);
  if (!strcmp(topic, "bibberBot/kp")) Kp = atof(val);
  if (!strcmp(topic, "bibberBot/ki")) Ki = atof(val);
  if (!strcmp(topic, "bibberBot/kd")) Kd = atof(val);
  unsigned long d = millis() - m;
  Serial.println(d);

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
  }
  return client.connected();

}
