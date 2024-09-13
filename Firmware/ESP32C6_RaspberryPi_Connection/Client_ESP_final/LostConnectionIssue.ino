#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
//#include <ESP32Servo.h>
#include <Servo.h>


// MODE-RED (Platform for controlling devices using MQTT)

// WiFi credentials
const char* ssid = "Mateinfo";   // WiFi network SSID
const char* password = "computer";  // WiFi network password
// const char* ssid = "RaspBerry"; //Hotspot SSID
// const char* password = "";

// MQTT Broker IP address and credentials
const char* mqtt_server = "172.30.245.94";  // MQTT broker IP for Wifi
//const char* mqtt_server = "10.42.0.1";  // MQTT broker IP for Hotspot
const char* mqtt_username = "telecomanda";  // MQTT username
const char* mqtt_password = "ABCabc";       // MQTT password



WiFiClient espClient;            // WiFi client
PubSubClient client(espClient);  // MQTT client object
long lastMsg = 0;                // Stores the last message time
char msg[50];                    // Buffer for MQTT message content

const unsigned long CONNECTION_TIMEOUT = 10000; // 10 seconds
unsigned long lastConnectionTime = 0;

// Motor Control Pins
const int ledPinLeft = 3;   // Pin for left motor
const int ledPinRight = 6;  // Pin for right motor


// --------------------------------------------------------------------


//SETUP


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

// Function to configure MQTT client
void setup_client() {
  client.setServer(mqtt_server, 1883);  // Set the MQTT server
  client.setCallback(callback);  // Set callback function for message reception
}

void setup_motors(){

  pinMode(ledPinLeft, OUTPUT);
  pinMode(ledPinRight, OUTPUT);

   // Start with motors off
  analogWrite(ledPinLeft, 0); 
  analogWrite(ledPinRight, 0);
}



void setup() {
  Serial.begin(9600);
  delay(2000);
  setup_wifi();
  setup_client();
  setup_motors();

}

void topic_motor_right(String& message){
  Serial.print("Changing output to ");
  if(message == "0"){
    Serial.println("RIGHT: off");
    analogWrite(ledPinRight, 0);
  }
  else if(message == "1")
  {
    Serial.println("RIGHT: on - medium");
    analogWrite(ledPinRight, 50);
    message = "0";
  }
  else if(message == "2")
  {
    Serial.println("RIGHT: on - high");
    analogWrite(ledPinRight, 100);
    message = "0";
  }
  else if(message == "3")
  {
    Serial.println("RIGHT: on - high");
    analogWrite(ledPinRight, 150);
    message = "0";
  }
  else if(message == "4")
  {
    Serial.println("RIGHT: on - high");
    analogWrite(ledPinRight, 200);
    message = "0";
  } 
}


void topic_motor_left(String& message)
{
  Serial.print("Changing output to ");
  if(message == "0"){
    Serial.println("LEFT: off");
    analogWrite(ledPinLeft, 0);
  }
  else if(message == "1")
  {
    Serial.println("RIGHT: on - medium");
    analogWrite(ledPinLeft, 50);
    client.publish("note/left", "Power: 50/255");
    message = "0";
  } 
  else if(message == "2")
  {
    Serial.println("RIGHT: on - high");
    analogWrite(ledPinLeft, 100);
    client.publish("note/left", "Power: 100/255");
    message = "0";
  }
  else if(message == "3")
  {
    Serial.println("RIGHT: on - high");
    analogWrite(ledPinLeft, 150);
    client.publish("note/left", "Power: 150/255");
    message = "0";
  }
  else if(message == "4")
  {
    Serial.println("RIGHT: on - high");
    analogWrite(ledPinLeft, 200);
    client.publish("note/left", "Power: 200/255");
    message = "0";
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);
  Serial.println("------------------");

  // Control GPIOs with MQTT using topics
  if(String(topic)=="motor/right")
  {
    topic_motor_right(messageTemp);
  }
  else if(String(topic)=="motor/left")
  {
    topic_motor_left(messageTemp);
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    delay(100);
    // Attempt to connect with username and password
    if (client.connect("ESP8266Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");

      // Subscribe to your topics
      client.subscribe("motor/left");
      client.subscribe("motor/right");

      // update last connection
      lastConnectionTime = millis();
 
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);

      if (millis() - lastConnectionTime > CONNECTION_TIMEOUT) {
        Serial.println("Connection timeout, stopping motors");
        delay(100); 
        //stopping motors
        analogWrite(ledPinRight, 0);
        analogWrite(ledPinLeft, 0);

      }
    }
  }
}





void loop() {

  if (!client.connected()) {
    reconnect();
  }
  else{
    lastConnectionTime = millis();
  }

  client.loop();

}