#include <Wire.h>

// Define constants and MPU6050 settings
const int MPU6050_ADDRESS = 0x68; // Standard I2C address for MPU6050
const int DLPF_CONFIG_REGISTER = 0x1A;   // Register for Digital Low Pass Filter configuration
const int GYRO_CONFIG_REGISTER = 0x1B;
const int ACCEL_CONFIG_REGISTER = 0x1C;
const int POWER_MANAGEMENT_REGISTER = 0x6B;
const int ACCEL_DATA_ADDRESS = 0x3B;
const int GYRO_DATA_ADDRESS = 0x43;

// Conversion factors for accelerometer and gyroscope
const float ACCEL_SENSITIVITY = 4096.0; // Sensitivity for ±8g range (g)
const float GYRO_SENSITIVITY = 65.5;    // Sensitivity for ±500°/s range (°/s)

// Offsets to account for sensor bias (calibration)
const float ACC_X_ERROR_CORRECTION = -0.04;
const float ACC_Y_ERROR_CORRECTION = 0;
const float ACC_Z_ERROR_CORRECTION = -0.01;

// Variables to store sensor data
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw = 0; // Calculated angles for Roll, Pitch, and Yaw (degrees)
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
const int CalibrationNumber = 4000;
// int PrintedAngleRoll, PrintedAnglePitch, PrintedAngleYaw;

const float alpha = 0.98;
unsigned long lastTime;


void setupMPU6050(){

  // Initialize MPU6050 by waking it up (writing to PWR_MGMT_1 register)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(POWER_MANAGEMENT_REGISTER);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Wake up the MPU6050 by setting sleep mode to 0
    Wire.endTransmission();

  // Set the Digital Low Pass Filter to 44 hz
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(DLPF_CONFIG_REGISTER);
    Wire.write(0x05);
    Wire.endTransmission();

    // Set the accelerometer range to ±8g (write to ACCEL_CONFIG register)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(ACCEL_CONFIG_REGISTER);
    Wire.write(0x10); // Set accelerometer sensitivity to ±8g
    Wire.endTransmission();

    // Set the gyroscope range to ±500°/s (write to GYRO_CONFIG register)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(GYRO_CONFIG_REGISTER);
    Wire.write(0x08); // Set gyroscope sensitivity to ±500°/s
    Wire.endTransmission();

  
}


// Function to read sensor data and calculate angles
void getMPU6050Data(void) {


    // Start reading accelerometer data (from register 0x3B)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(ACCEL_DATA_ADDRESS); // Starting address for accelerometer data
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 6); // Request 6 bytes for X, Y, and Z accelerations

    // Read the accelerometer values
    int16_t AccXLSB = Wire.read() << 8 | Wire.read(); // X acceleration
    int16_t AccYLSB = Wire.read() << 8 | Wire.read(); // Y acceleration
    int16_t AccZLSB = Wire.read() << 8 | Wire.read(); // Z acceleration

    // Convert accelerometer raw data to g (acceleration)
    AccX = (float)AccXLSB / ACCEL_SENSITIVITY + ACC_X_ERROR_CORRECTION; // X-axis acceleration
    AccY = (float)AccYLSB / ACCEL_SENSITIVITY + ACC_Y_ERROR_CORRECTION;                // Y-axis acceleration
    AccZ = (float)AccZLSB / ACCEL_SENSITIVITY + ACC_Z_ERROR_CORRECTION; // Z-axis acceleration


    // Start reading gyroscope data (from register 0x43)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(GYRO_DATA_ADDRESS); // Starting address for gyroscope data
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 6); // Request 6 bytes for X, Y, and Z rotations

    // Read the gyroscope values
    int16_t GyroX = Wire.read() << 8 | Wire.read(); // X gyro
    int16_t GyroY = Wire.read() << 8 | Wire.read(); // Y gyro
    int16_t GyroZ = Wire.read() << 8 | Wire.read(); // Z gyro

    // Convert gyroscope raw data to °/s (angular velocity)
    RateRoll = (float)GyroX / GYRO_SENSITIVITY;   // X-axis rate
    RatePitch = (float)GyroY / GYRO_SENSITIVITY;  // Y-axis rate
    RateYaw = (float)GyroZ / GYRO_SENSITIVITY;    // Z-axis rate

    //Calculate roll and pitch angles using accelerometer data
  // Roll (rotation around X-axis)
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  // Pitch (rotation around Y-axis)
  AnglePitch = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;

}


void calibrateSensorsRate(){
  
  // Calibration: gather initial data
    for (int RateCalibrationNumber = 0; RateCalibrationNumber < CalibrationNumber; RateCalibrationNumber++) {
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


void setup() {
    Serial.begin(57600);      // Start serial communication
    Wire.setClock(400000);    // Set I2C speed to 400kHz
    Wire.begin();             // Initialize I2C communication
    delay(250);               // Small delay for stability

    setupMPU6050();
    calibrateSensorsRate();
    lastTime = millis();

}

void loop() {

  getMPU6050Data();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;

  AngleRoll = alpha * (AngleRoll + RateRoll * deltaTime) + (1 - alpha) * AngleRoll;
  AnglePitch = alpha * (AnglePitch + RatePitch * deltaTime) + (1 - alpha) * AnglePitch;


  //Integrate the yaw rate to estimate yaw angle
  float ChangedAngleYaw = RateYaw * deltaTime;
  if(abs(ChangedAngleYaw) > 0.01){
    AngleYaw += ChangedAngleYaw;
  }

  // Print the calculated angles to the serial monitor

  // if(abs(PrintedAngleRoll - AngleRoll) > 1){
  //   PrintedAngleRoll = AngleRoll;
  // }
  // if(abs(PrintedAnglePitch - AnglePitch) > 1){
  //   PrintedAnglePitch = AnglePitch;
  // }
  // if(abs(PrintedAngleYaw - AngleYaw) > 1){
  //   PrintedAngleYaw = AngleYaw;
  // }
  // Serial.print(" Roll Angle [degrees]= ");
  // Serial.print(PrintedAngleRoll);
  // Serial.print(" Pitch Angle [degrees]= ");
  // Serial.print(PrintedAnglePitch);
  // Serial.print(" Yaw Angle [degrees]= ");
  // Serial.println(PrintedAngleYaw);

  Serial.print("Roll Angle [degrees]= ");
  Serial.print(AngleRoll);
  Serial.print(" Pitch Angle [degrees]= ");
  Serial.print(AnglePitch);
  Serial.print(" Yaw Angle [degrees]= ");
  Serial.println(AngleYaw);

  // Update the loop timer
  delay(500);
}