#include <WiFiLink.h>
#include <PubSubClient.h>
#include "secret.h";
#include <stdlib.h>

const char* mqtt_server = "broker.mqtt-dashboard.com";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long lastReconnectAttempt = 0;
char msg[50];
int value = 0;

void setup() {
  pinMode(9, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(38, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
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
  if (!strcmp(topic, "bibberBot/speed")) Serial.println("speed:");
  if (!strcmp(topic, "bibberBot/turn")) Serial.println("turn:");
  Serial.println(atoi(val));
}


boolean reconnect() {
  if (client.connect("bibberBot")) {
    Serial.println("connected");
    client.publish("thomasOut", "hello world");
    client.subscribe("bibberBot/speed");
    client.subscribe("bibberBot/turn");
  }
  return client.connected();
}


void loop() {
  long now = millis();
  if (!client.connected()) {
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    client.loop();
    if (now - lastMsg > 2000) {
      lastMsg = now;
      ++value;
      snprintf (msg, 75, "hello world #%ld", value);
      Serial.print("Publish message: ");
      Serial.println(msg);
      client.publish("outTopic", msg);
    }
  }

}
