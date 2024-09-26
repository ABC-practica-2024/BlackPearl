#define TRIG_PIN 9
#define ECHO_PIN 10

void setup() {
  
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLUP);
  digitalWrite(ECHO_PIN, HIGH);
}

void loop() {
  // Clear the Trig pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the Trig pin high for 10 microseconds to start measurement
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the Echo pin
  long duration = pulseIn(ECHO_PIN, HIGH, 26000); // returns the duration of a pulse in microseconds
  
  // Calculate distance in cm
  float distance = (duration / 2.0) * 0.1484; // Speed of Sound in Water: 1,484 meters per second = 148,400 cm per second, or 0.1484 cm per microsecond.

  if(distance < 25){
    Serial.println("Ancora este ridicata");
  }else{
    Serial.println("Ancora este coborata");
  }
  
  delay(500);
}
