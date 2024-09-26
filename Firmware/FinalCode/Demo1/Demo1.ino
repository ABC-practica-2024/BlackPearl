#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
//#include <ESP32Servo.h>
#include <Servo.h>
#include "qmc5883l.h"


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


// Servo 

const int servoPin = 9;    // Pin for servo
Servo ESC;                 // Servo object to control the ESC (Electronic Speed Controller)
int throttle = 90;         // Default throttle value
bool isPulling = false;    // If the servo is pulling the box

// ---------------------------------------------------------------------



// MPU6050

// MPU6050 I2C address and register definitions
const int MPU6050_ADDRESS = 0x68;         // I2C address of MPU6050
const int DLPF_CONFIG_REGISTER = 0x1A;    // Digital Low Pass Filter configuration register
const int GYRO_CONFIG_REGISTER = 0x1B;    // Gyroscope configuration register
const int ACCEL_CONFIG_REGISTER = 0x1C;   // Accelerometer configuration register
const int POWER_MANAGEMENT_REGISTER = 0x6B;  // Power management register
const int ACCEL_DATA_ADDRESS = 0x3B;      // Starting address for accelerometer data
const int GYRO_DATA_ADDRESS = 0x43;       // Starting address for gyroscope data

// Sensor sensitivity conversion factors
const float ACCEL_SENSITIVITY = 4096.0;  // Accelerometer sensitivity for ±8g range (g)
const float GYRO_SENSITIVITY = 65.5;     // Gyroscope sensitivity for ±500°/s range (°/s)

// Calibration offsets for accelerometer
const float ACC_X_ERROR_CORRECTION = -0.04; // X-axis bias correction
const float ACC_Y_ERROR_CORRECTION = 0;     // Y-axis bias correction
const float ACC_Z_ERROR_CORRECTION = -0.01; // Z-axis bias correction

// Variables to hold sensor data and angles
float AccX, AccY, AccZ;   // Accelerometer values
float AngleRoll, AnglePitch, AngleYaw = 0; // Calculated angles (degrees)
float RateRoll, RatePitch, RateYaw;        // Gyroscope angular rates
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;  // Calibration offsets
const int CalibrationNumber = 2000;  // Number of samples for calibration
int PrintedAngleRoll, PrintedAnglePitch, PrintedAngleYaw;  // Angles for output

const float alpha = 0.98;  // Complementary filter constant
unsigned long lastTime, printTime;  // Time variables for tracking loop timing

// ---------------------------------------------------------------------



// Magnetometer 

boolean error;

QMC5883L qmc5883l(&Wire);

// ---------------------------------------------------------------------

// Senzor ultrasonic

#define TRIG_PIN 10
#define ECHO_PIN 11
bool boxIsUp = true;


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

// Function to initialize and calibrate servo motor (ESC)
void setup_servo() {
  pinMode(servoPin, OUTPUT);
  ESC.attach(servoPin, 1000, 2000);  // Attach servo with calibration pulse width
  ESC.write(throttle);  // Initialize throttle
  delay(2000);  // Allow servo to stabilize
}



void setupMPU6050() {


  // Wake up MPU6050 by clearing sleep mode
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(POWER_MANAGEMENT_REGISTER);
  Wire.write(0x00);  // Set sleep mode to 0
  Wire.endTransmission();

  // Configure Digital Low Pass Filter to 44Hz
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(DLPF_CONFIG_REGISTER);
  Wire.write(0x05);  // Set to 44Hz
  Wire.endTransmission();

  // Set accelerometer to ±8g range
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_CONFIG_REGISTER);
  Wire.write(0x10);  // ±8g range
  Wire.endTransmission();

  // Set gyroscope to ±500°/s range
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYRO_CONFIG_REGISTER);
  Wire.write(0x08);  // ±500°/s range
  Wire.endTransmission();
}

void setupMagnetometer(){
  if (!qmc5883l.begin()) error = true;
   else
   {
      qmc5883l.setConfig({ QMC5883L::Mode::continuous,
                           QMC5883L::OutputDataRate::odr_10hz,
                           QMC5883L::FullScaleRange::rng_8g,
                           QMC5883L::OverSampleRate::osr_512,
                           false                              });

      if (!qmc5883l.writeConfig()) error = true;
      else
      {
         qmc5883l.setCalibration({ {255.120624, 
                                    -449.527586,
                                     9.694871 },
                                  {{   1.274925,   -0.053847,  -0.006444},
                                   {  -0.053847,    1.206629,   0.056813},
                                   {  -0.006444,    0.056813,   1.318348} }});

         error = false;
      }
   }
}


void setupUltrasonicSensor(){

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLUP);
  
}


void setup() {
  Serial.begin(9600);
  delay(2000);
  setup_wifi();
  setup_client();
  setup_motors();
  setup_servo();

  Wire.setClock(400000);      // Set I2C clock to 400kHz
  Wire.begin();               // Initialize I2C bus
  delay(250);                 // Wait for sensor stabilization
  setupMPU6050();
  setupMagnetometer();
  setupUltrasonicSensor();

}



// Function to calibrate gyroscope by averaging data over multiple samples
void calibrateSensorsRate() {
  for (int i = 0; i < CalibrationNumber; i++) {
      getMPU6050Data();
      RateCalibrationRoll += RateRoll;
      RateCalibrationPitch += RatePitch;
      RateCalibrationYaw += RateYaw;
      delay(1);
  }
  RateCalibrationRoll /= CalibrationNumber;
  RateCalibrationPitch /= CalibrationNumber;
  RateCalibrationYaw /= CalibrationNumber;
}


void Topic_motor_right(String& message){
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


void Topic_motor_left(String& message)
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

void Topic_servo(String& message){

  if(message == "stop")
  {
    throttle = 90;
    isPulling = false;
  }
  else if(message == "pull" && isPulling == false){
    throttle = 105;
    isPulling = true;
  }
  else if(message == "drop"){
    throttle = 85;
    ESC.write(throttle);
    delay(100);
    ESC.write(90); //it's the only way that works
    delay(20);
    isPulling = false;
      
  }
  Serial.print("Changed to: ");
  Serial.println(throttle);
  ESC.write(throttle);
  
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
    Topic_motor_right(messageTemp);
  }
  else if(String(topic)=="motor/left")
  {
    Topic_motor_left(messageTemp);
  }
  else if(String(topic) == "servo")
  {
    Topic_servo(messageTemp);
  }
}






void getMPU6050Data() {
  // Read accelerometer data (6 bytes for X, Y, Z axes)
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_DATA_ADDRESS);  // Starting address for accelerometer data
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 6);  // Request 6 bytes
    
  // Convert raw accelerometer data to g (gravity)
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();  // X-axis
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();  // Y-axis
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();  // Z-axis
  AccX = (float)AccXLSB / ACCEL_SENSITIVITY + ACC_X_ERROR_CORRECTION;  // X-axis in g
  AccY = (float)AccYLSB / ACCEL_SENSITIVITY + ACC_Y_ERROR_CORRECTION;  // Y-axis in g
  AccZ = (float)AccZLSB / ACCEL_SENSITIVITY + ACC_Z_ERROR_CORRECTION;  // Z-axis in g

  // Read gyroscope data (6 bytes for X, Y, Z axes)
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYRO_DATA_ADDRESS);  // Starting address for gyroscope data
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS, 6);  // Request 6 bytes
    
  // Convert raw gyroscope data to degrees per second
  int16_t GyroX = Wire.read() << 8 | Wire.read();  // X-axis
  int16_t GyroY = Wire.read() << 8 | Wire.read();  // Y-axis
  int16_t GyroZ = Wire.read() << 8 | Wire.read();  // Z-axis
  RateRoll = (float)GyroX / GYRO_SENSITIVITY;    // X-axis rate (roll)
  RatePitch = (float)GyroY / GYRO_SENSITIVITY;   // Y-axis rate (pitch)
  RateYaw = (float)GyroZ / GYRO_SENSITIVITY;     // Z-axis rate (yaw)

  // Calculate angles using accelerometer data
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;   // Roll angle
  AnglePitch = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;  // Pitch angle
}

// Function to apply complementary filter and calculate angles
void getAngles() {
  // Subtract calibration offsets
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Calculate time delta in seconds
  float deltaTime = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  // Apply complementary filter to compute roll and pitch angles
  AngleRoll = alpha * (AngleRoll + RateRoll * deltaTime) + (1 - alpha) * AngleRoll;
  AnglePitch = alpha * (AnglePitch + RatePitch * deltaTime) + (1 - alpha) * AnglePitch;

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
      client.subscribe("servo");

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
        ESC.write(90);

      }
    }
  }
}



void sendMPU6050Data(){
  // Only update printed angles if significant change is detected
  if (abs(PrintedAngleRoll - AngleRoll) > 2) {
      PrintedAngleRoll = AngleRoll;
      Serial.print("Angle Roll: ");
      Serial.println(PrintedAngleRoll);
      delay(100);
      client.publish("angle/roll", String(PrintedAngleRoll).c_str());
      Serial.println("Sent");
      delay(100);
  }
  if (abs(PrintedAnglePitch - AnglePitch) > 2) {
      PrintedAnglePitch = AnglePitch;
      Serial.print("Angle Pitch: ");
      Serial.println(PrintedAnglePitch);
      delay(100);
      client.publish("angle/pitch", String(PrintedAnglePitch).c_str());
      Serial.println("Sent");
      delay(100);
  }
  


}

void loopMPU6050(){
  getMPU6050Data();   // Read sensor data
  getAngles();        // Calculate angles using complementary filter
  sendMPU6050Data();
  
}

void loopMagnetometer(){
  //if (error) return;
   

   if (qmc5883l.readData())
   {
      AngleYaw = qmc5883l.azimuthZUp();
      if (abs(PrintedAngleYaw - AngleYaw) > 2) {
      PrintedAngleYaw = AngleYaw;
      Serial.print("Angle Yaw: ");
      Serial.println(PrintedAngleYaw);
      delay(100);
      client.publish("angle/yaw", String(PrintedAngleYaw).c_str());
      Serial.println("Sent");
      delay(100);
    }
   }
}


void checkUltrasonicSensor(){

  boxIsUp = false;
  // Clear the Trig pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the Trig pin high for 10 microseconds to start measurement
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the Echo pin
  long duration = pulseIn(ECHO_PIN, HIGH, 1350); // returns the duration of a pulse in microseconds for maximum 1m under water
  
  // Calculate distance in cm
  float distance = (duration / 2.0) * 0.1484; // Speed of Sound in Water: 1,484 meters per second = 148,400 cm per second, or 0.1484 cm per microsecond.

  if(distance < 25){
    boxIsUp = true;
  }

}


void loopUltrasonicSensor(){

  checkUltrasonicSensor();
  if(isPulling == true && boxIsUp == true)
  {
    ESC.write(90);
    boxIsUp = false;
    isPulling = false;
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
  

  // Send angles every 100ms
  if (millis() - printTime > 100) {
      printTime = millis();
      loopMPU6050();
      loopMagnetometer();
      
  }

  loopUltrasonicSensor(); 

}



void printErrorMagnetometer(QMC5883L::Error error, char *error_location)
{
   Serial.print("Error: ");
   switch(error)
   {
      case QMC5883L::i2c_buffer_overflow: Serial.print("I2C buffer overflow");          break;
      case QMC5883L::i2c_address_nack:    Serial.print("I2C address not acknowledged"); break;
      case QMC5883L::i2c_data_nack:       Serial.print("I2C data not acknowledged");    break;
      case QMC5883L::i2c_other:           Serial.print("I2C other");                    break;
      case QMC5883L::i2c_request_partial: Serial.print("I2C request partial");          break;
      case QMC5883L::qmc_data_overflow:   Serial.print("QMC5883L data overflow");       break;
   }
   Serial.print(error);
   Serial.print(" in ");
   Serial.println(error_location);
}