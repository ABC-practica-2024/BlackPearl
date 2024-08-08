#include <Arduino_MKRGPS.h>

int i=0;

int nr = 0;

float avg_lat = 0;
float avg_long = 0;
float avg_alt = 0;

void setup() {
  // initialize serial communications and wait for port to open:
  
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // If you are using the MKR GPS as shield, change the next line to pass
  // the GPS_MODE_SHIELD parameter to the GPS.begin(...)
  if (!GPS.begin()) {
    Serial.println("Failed to initialize GPS!");
    while (1);
  }
}

void loop() {
  // check if there is new GPS data available
  if (i == 30)
  {
    avg_lat = avg_lat/nr;
    avg_long = avg_long/nr;
    avg_alt = avg_alt/nr;

    Serial.print("Location: ");
    Serial.print(avg_lat, 7);
    Serial.print(", ");
    Serial.println(avg_long, 7);

    Serial.print("Altitude: ");
    Serial.print(avg_alt);
    Serial.println("m");

    Serial.println();

    avg_lat = 0;
    avg_long = 0;
    avg_alt = 0;

    i=0;

    nr = 0;
  }
  if (GPS.available()) {
    // read GPS values
    float latitude   = GPS.latitude();
    float longitude  = GPS.longitude();
    float altitude   = GPS.altitude(); 

    int satellites = GPS.satellites();

    avg_lat += satellites * latitude;
    avg_long += satellites * longitude;
    avg_alt += satellites * altitude;

    nr += satellites;

    i++;
  }
}