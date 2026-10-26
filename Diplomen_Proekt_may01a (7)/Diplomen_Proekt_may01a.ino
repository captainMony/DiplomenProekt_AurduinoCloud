#include "arduino_secrets.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME680.h>
#include "thingProperties.h" // Include the generated thingProperties.h file
#include <Wire.h>

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BME680 bme;

// Constants for calculations
const float GAS_RESISTANCE_MIN = 1000.0;    // Minimum gas resistance in ohms
const float GAS_RESISTANCE_MAX = 200000.0;  // Maximum gas resistance in ohms
const float AQI_SCALE_MAX = 500.0;          // Maximum AQI scale value

// Program variable definitions with initial values
float ProgramVibrationThreshold = 10.0; // in m/s^2, must be higher than gravity's pull
float ProgramTiltThreshold = 10.0;      // in degrees
int ProgramMaxTemp = 100;               // Upper limit for temperature
int ProgramMinTemp = 0;                 // Lower limit for temperature
int ProgramMaxHumidity = 70;            // Upper limit for humidity (default)
bool ProgramAQVeryGood = true;
bool ProgramAQNormal = false;
bool ProgramAQVeryBad = false;

void setup() {
  Serial.begin(9600);
  delay(1500);

  // Start the BME688 module
  if (!bme.begin(0x76)) {
    Serial.println("Sensor not found at 0x76");
    while (1);
  }
  Serial.println("BME688 Started");

  // Start the MPU6050 module
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Started");

  // Configure MPU sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("MPU configured");
  delay(100);

  // Configure BME sensor
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150ms
  Serial.println("BME configured");

  // Initialize IoT Cloud properties
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection); // Start cloud connection
  delay(3000);

  // Debugging information for IoT Cloud connection
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  // Update the cloud
  ArduinoCloud.update();
  delay(2000);

  // Call the function to read vibration and tilt
  detectVibrationAndTilt();
  delay(500);

  // Call the function to read temperature, humidity, and air quality
  readBmeSensor();
  delay(500);

  // Call the function to invert cloud warnings for visual clarity 
  invertCloudWarnings();
  
}

// Subprograms ----------------------------------------------------------------------------------------------

void detectVibrationAndTilt() { // Get new sensor events
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Vibration detection
  float totalAcceleration = sqrt(accel.acceleration.x * accel.acceleration.x +
                                  accel.acceleration.y * accel.acceleration.y +
                                  accel.acceleration.z * accel.acceleration.z);

  if (totalAcceleration > ProgramVibrationThreshold) {
    cloudVibrationWarning = true; // Set the indicator
    Serial.println("Vibration Detected!");
    Serial.print("Total Acceleration: ");
    Serial.println(totalAcceleration);
  } else {
    cloudVibrationWarning = false; // Clear the indicator
  }

  // Tilt detection
  float tiltX = atan2(accel.acceleration.y,
                      sqrt(accel.acceleration.x * accel.acceleration.x +
                           accel.acceleration.z * accel.acceleration.z)) *
                180 / PI;
  float tiltY = atan2(-accel.acceleration.x,
                      sqrt(accel.acceleration.y * accel.acceleration.y +
                           accel.acceleration.z * accel.acceleration.z)) *
                180 / PI;

  // Update cloud variables
  cloudTiltX = tiltX; // Live view
  cloudTiltY = tiltY;
  cloudLiveVibrations = totalAcceleration;

  // Check tilt thresholds and display warnings
  if (abs(tiltX) > ProgramTiltThreshold || abs(tiltY) > ProgramTiltThreshold) {
    cloudTiltWarning = true; // Set the indicator
    Serial.println("-------------MPU6050--------------------");
    Serial.println("Tilt Detected!");
    Serial.print("Tilt X: ");
    Serial.println(tiltX);
    Serial.print("Tilt Y: ");
    Serial.println(tiltY);
    Serial.println("---------------------------------");
  } else {
    cloudTiltWarning = false; // Clear the indicator
  }
}

void readBmeSensor() {
  // Perform a sensor reading
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading");
    return;
  }

  bME_Temp = bme.temperature; // Display all parameters
  bME_Index = calculateAQI(bme.gas_resistance, bme.humidity);
  bME_Humidity = bme.humidity;

  // Raise AQI, temperature, and humidity indicators
  raiseAQIndicators();
  raiseTemperatureIndicators();
  raiseHumidityIndicators();

  // Print to Serial Monitor when needed
  // SerialMonitorDebuggingBME();
}

void raiseAQIndicators() {
  // Reset all AQI indicators to false initially
  cloudAQVeryGood = false;
  cloudAQNormal = false;
  cloudAQVeryBad = false;

  // Check conditions and raise the appropriate indicator
  if (bME_Index >= 0 && bME_Index <= 100) {
    cloudAQVeryGood = true; // Very Good air quality
    Serial.println("Air Quality: Very Good");
  } else if (bME_Index > 100 && bME_Index <= 220) {
    cloudAQNormal = true; // Normal air quality
    Serial.println("Air Quality: Normal");
  } else if (bME_Index > 220 && bME_Index <= 500) {
    cloudAQVeryBad = true; // Very Bad air quality
    Serial.println("Air Quality: Very Bad");
  } else {
    Serial.println("Air Quality Index out of range");
  }
}

void raiseTemperatureIndicators() {
  // Reset all temperature indicators to false initially
  cloudTempWarning = false;
  cloudTooLowTempWarning = false;

  // Check temperature against the limits and raise appropriate indicators
  if (bME_Temp > ProgramMaxTemp) {
    cloudTempWarning = true; // Temperature exceeded upper limit
    Serial.println("WARNING: Temperature exceeded the upper limit!");
  } else if (bME_Temp < ProgramMinTemp) {
    cloudTooLowTempWarning = true; // Temperature fell below lower limit
    Serial.println("WARNING: Temperature fell below the lower limit!");
  } else {
    Serial.println("Temperature is within normal limits.");
  }
}

void raiseHumidityIndicators() {
  // Reset the humidity warning
  cloudHumidityWarning = false;

  // Check if the humidity is above the configured upper limit
  if (bME_Humidity > ProgramMaxHumidity) {
    cloudHumidityWarning = true; // Raise humidity warning
    Serial.println("WARNING: Humidity is above the configured upper limit!");
  } else {
    Serial.println("Humidity is within the configured range.");
  }
}

void SerialMonitorDebuggingBME() {
  Serial.println("--------------BME688-------------------");
  Serial.print("Temperature: ");
  Serial.print(bme.temperature);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(bme.humidity);
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(bme.pressure / 100.0); // hPa
  Serial.println(" hPa");
  Serial.print("Gas Resistance: ");
  Serial.print(bme.gas_resistance / 1000.0); // kOhms
  Serial.println(" kOhm");
  Serial.println("---------------------------------");
}

void invertCloudWarnings() {
  cloudTempWarning = !cloudTempWarning;
  cloudTiltWarning = !cloudTiltWarning;
  cloudTooLowTempWarning = !cloudTooLowTempWarning;
  cloudVibrationWarning = !cloudVibrationWarning;
  cloudHumidityWarning = !cloudHumidityWarning;
}

// Function to calculate approximate Air Quality Index (AQI)
float calculateAQI(float gasResistance, float humidity) {
  float aqi = 0.0;

  // Normalize gas resistance to a logarithmic scale
  float gasScore = (log(gasResistance) - log(GAS_RESISTANCE_MIN)) /
                   (log(GAS_RESISTANCE_MAX) - log(GAS_RESISTANCE_MIN));
  gasScore = constrain(gasScore, 0, 1); // Ensure the gas score is between 0 and 1
  gasScore = gasScore * AQI_SCALE_MAX;  // Scale to IAQ range

  // Normalize humidity to a comfort range
  float humidityScore = 0;
  if (humidity > ProgramMaxHumidity) {
    humidityScore = (humidity - ProgramMaxHumidity) / ProgramMaxHumidity;
  }
  humidityScore = humidityScore * AQI_SCALE_MAX;

  // Combine gas resistance and humidity scores
  aqi = (gasScore + humidityScore) / 2; // Average the two scores

  return aqi; // Return the Air Quality Index (scaled to 0-500)
}

// Cloud inputs ----------------------------------------------------------------------------------------------
void onCloudTempUpperLimitInputChange() {
  if (cloudTemp_Upper_Limit_input != ProgramMaxTemp) {
    ProgramMaxTemp = cloudTemp_Upper_Limit_input;
    Serial.println("TempUpperLimitChanged");
  }
}

void onCloudTempLowerLimitInputChange() {
  if (cloudTemp_Lower_Limit_input != ProgramMinTemp) {
    ProgramMinTemp = cloudTemp_Lower_Limit_input;
    Serial.println("TempLowerLimitChanged");
  }
}

void onCloudVibrationUpperLimitInputChange() {
  if (cloud_Vibration_Upper_Limit_input != ProgramVibrationThreshold) {
    ProgramVibrationThreshold = cloud_Vibration_Upper_Limit_input;
    Serial.println("VibrationUpperLimitChanged");
  }
}

void onCloudHumidityUpperLimitInputChange() {
  if (cloudHumidity_Upper_Limit_input != ProgramMaxHumidity) {
    ProgramMaxHumidity = cloudHumidity_Upper_Limit_input;
    Serial.println("HumidityUpperLimitChanged");
  }
}
/*
  Since CloudTiltLimitInput is READ_WRITE variable, onCloudTiltLimitInputChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onCloudTiltLimitInputChange() {
  if (cloudTilt_Limit_input != ProgramTiltThreshold) {
    ProgramTiltThreshold = cloudTilt_Limit_input; 
    Serial.println("TiltLimitChanged");          
  }
}