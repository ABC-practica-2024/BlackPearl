#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
//COD BUN

// Your SSID and password
// const char* ssid = "Mateinfo";
// const char* password = "computer";

const char* ssid = "RaspBerry";
const char* password = "";

//MQTT Broker IP address
//const char* mqtt_server = "172.30.245.94";
const char* mqtt_server = "10.42.0.1";

//MQTT Broker username and password
const char* mqtt_username = "telecomanda";
const char* mqtt_password = "ABCabc";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// Motor Controll Pins
const int ledPinLeft = 3;
const int ledPinRight = 6;

//initial setup
void setup() {
  Serial.begin(921600);
  delay(2000);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(ledPinLeft, OUTPUT);
  pinMode(ledPinRight, OUTPUT);
  analogWrite(ledPinLeft, 0);
  analogWrite(ledPinRight, 0);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //start the connection
  WiFi.begin(ssid, password);

  //chack the wifi status
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if(WiFi.status() == WL_CONNECT_FAILED)
    {
      Serial.print(" CONNECTION FAILED WOMP ");
    }
    if(WiFi.status() == WL_NO_SSID_AVAIL)
      {
        Serial.print("  No ssid available  ");
      }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  Serial.println(messageTemp);
  Serial.println("------------------");

  // Control GPIOs with MQTT using topics
  if(String(topic)=="motor/right")
  {
    Serial.print("Changing output to ");
    if(messageTemp == "0"){
      Serial.println("RIGHT: off");
      analogWrite(ledPinRight, 0);
    }
    else if(messageTemp == "1")
    {
      Serial.println("RIGHT: on - medium");
      analogWrite(ledPinRight, 50);
      messageTemp = "0";
    }
    else if(messageTemp == "2")
    {
      Serial.println("RIGHT: on - high");
      analogWrite(ledPinRight, 100);
      messageTemp = "0";
    }
    else if(messageTemp == "3")
    {
      Serial.println("RIGHT: on - high");
      analogWrite(ledPinRight, 150);
      messageTemp = "0";
    }
    else if(messageTemp == "4")
    {
      Serial.println("RIGHT: on - high");
      analogWrite(ledPinRight, 200);
      messageTemp = "0";
    }
  }
  else if(String(topic)=="motor/left")
  {
    Serial.print("Changing output to ");
    if(messageTemp == "0"){
      Serial.println("LEFT: off");
      analogWrite(ledPinLeft, 0);
    }
    else if(messageTemp == "1")
    {
      Serial.println("RIGHT: on - medium");
      analogWrite(ledPinLeft, 50);
      client.publish("note/left", "Power: 50/255");
      messageTemp = "0";
    }
    else if(messageTemp == "2")
    {
      Serial.println("RIGHT: on - high");
      analogWrite(ledPinLeft, 100);
      client.publish("note/left", "Power: 100/255");
      messageTemp = "0";
    }
    else if(messageTemp == "3")
    {
      Serial.println("RIGHT: on - high");
      analogWrite(ledPinLeft, 150);
      client.publish("note/left", "Power: 150/255");
      messageTemp = "0";
    }
    else if(messageTemp == "4")
    {
      Serial.println("RIGHT: on - high");
      analogWrite(ledPinLeft, 200);
      client.publish("note/left", "Power: 200/255");
      messageTemp = "0";
    }
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect with username and password
    if (client.connect("ESP8266Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Subscribe to your topics
      client.subscribe("motor/left");
      client.subscribe("motor/right");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  // ~~ test code for checking available wifi connections
  //  int n = WiFi.scanNetworks();
  // Serial.println("scan done");
  // if (n == 0) {
  //     Serial.println("no networks found");
  // } else {
  //   Serial.print(n);
  //   Serial.println(" networks found");
  //   for (int i = 0; i < n; ++i) {
  //     // Print SSID and RSSI for each network found
  //     Serial.print(i + 1);
  //     Serial.print(": ");
  //     Serial.print(WiFi.SSID(i));
  //     Serial.print(" (");
  //     Serial.print(WiFi.RSSI(i));
  //     Serial.print(")");
  //     Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
  //     delay(10);
  //   }
  // }
  // Serial.println("");

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    delay(100);
  }
}