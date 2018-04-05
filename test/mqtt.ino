#include <WiFiLink.h>
#include <PubSubClient.h>
#include "secret.h";
#include <stdlib.h>

const char* mqtt_server = "broker.mqtt-dashboard.com";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  pinMode(9, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(NETWORK);
  WiFi.begin(NETWORK,PASS);
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
  char val[length+1];
  for (int i=0;i<length;i++) {
    val[i] = (char)payload[i];
  }
  val[length+1] = '\0';
  if (!strcmp(topic, "bibberBot/speed")) Serial.println("speed:");
  if (!strcmp(topic, "bibberBot/turn")) Serial.println("turn:");
  Serial.println(atoi(val));
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("Bibberbot")) {
      Serial.println("connected");
      client.publish("thomasOut", "hello world");
      client.subscribe("bibberBot/speed");
      client.subscribe("bibberBot/turn");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, 75, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("outTopic", msg);
  }
}
