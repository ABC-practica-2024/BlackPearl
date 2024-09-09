#include <Wire.h>

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

// Function to configure MPU6050 settings
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

// Function to read data from the MPU6050 and convert to physical values
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

    // Integrate yaw rate to estimate yaw angle
    if (abs(RateYaw) > 0.1) {
        AngleYaw += RateYaw * deltaTime;
    }
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

void setup() {
    Serial.begin(57600);        // Initialize serial communication
    Wire.setClock(400000);      // Set I2C clock to 400kHz
    Wire.begin();               // Initialize I2C bus
    delay(250);                 // Wait for sensor stabilization

    setupMPU6050();             // Configure MPU6050
    calibrateSensorsRate();     // Calibrate gyroscope
    lastTime = millis();        // Set initial time
    printTime = millis();       // Set initial print time
}

void loop() {
    getMPU6050Data();   // Read sensor data
    getAngles();        // Calculate angles using complementary filter

    // Only update printed angles if significant change is detected
    if (abs(PrintedAngleRoll - AngleRoll) > 1) {
        PrintedAngleRoll = AngleRoll;
    }
    if (abs(PrintedAnglePitch - AnglePitch) > 1) {
        PrintedAnglePitch = AnglePitch;
    }
    if (abs(PrintedAngleYaw - AngleYaw) > 1) {
        PrintedAngleYaw = AngleYaw;
    }

    // Print angles every 300ms
    if (millis() - printTime > 300) {
        printTime = millis();
        Serial.print(" Roll Angle [degrees]= ");
        Serial.print(PrintedAngleRoll);
        Serial.print(" Pitch Angle [degrees]= ");
        Serial.print(PrintedAnglePitch);
        Serial.print(" Yaw Angle [degrees]= ");
        Serial.println(PrintedAngleYaw);
    } else {
        delay(13);  // Add small delay equivalent to the print time
    }
}
