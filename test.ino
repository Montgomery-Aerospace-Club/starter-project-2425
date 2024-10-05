#include <Wire.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;
MPU6050 mpu;

// Kalman filters for accelerometer and gyroscope data
SimpleKalmanFilter kalmanAccX(1, 1, 0.01);
SimpleKalmanFilter kalmanAccY(1, 1, 0.01);
SimpleKalmanFilter kalmanAccZ(1, 1, 0.01);

SimpleKalmanFilter kalmanAngleX(1, 1, 0.01);
SimpleKalmanFilter kalmanAngleY(1, 1, 0.01);

int16_t ax, ay, az;
int16_t gx, gy, gz;
float accX, accY, accZ;
float accXC = 0, accYC = 0, accZC = 0;  // Offset values for calibration
float filteredAccX, filteredAccY, filteredAccZ;
float tiltAngleX = 0, tiltAngleY = 0;    // Tilt angles in degrees
float velocityX = 0, velocityY = 0, velocityZ = 0;
float prevTime = 0;
float dt;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 connected successfully");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");

  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println("time,vel");
  dataFile.close();

  delay(2000);
  prevTime = millis();
}

void loop() {
  // Read raw acceleration and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw accelerometer values to g's (m/s^2)
  accX = (ax / 16384.0 - accXC) * 9.8;
  accY = (ay / 16384.0 - accYC) * 9.8;
  accZ = (az / 16384.0 - accZC) * 9.8;

  // Kalman filtering of acceleration data
  filteredAccX = kalmanAccX.updateEstimate(accX);
  filteredAccY = kalmanAccY.updateEstimate(accY);
  filteredAccZ = kalmanAccZ.updateEstimate(accZ);

  // Get current time and calculate dt
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Calculate angular velocity in degrees/s (Gyroscope values are in raw form)
  float gyroXrate = gx / 131.0;  // 131 LSB/°/s for MPU6050
  float gyroYrate = gy / 131.0;

  // Integrate gyroscope rates to get tilt angles
  tiltAngleX += gyroXrate * dt;
  tiltAngleY += gyroYrate * dt;

  // Use accelerometer data to calculate tilt angle from the horizontal (roll and pitch)
  float accelAngleX = atan2(filteredAccY, filteredAccZ) * 180 / PI;
  float accelAngleY = atan2(filteredAccX, filteredAccZ) * 180 / PI;

  // Apply Kalman filtering to the calculated angles
  tiltAngleX = kalmanAngleX.updateEstimate(tiltAngleX * 0.98 + accelAngleX * 0.02);
  tiltAngleY = kalmanAngleY.updateEstimate(tiltAngleY * 0.98 + accelAngleY * 0.02);

  // Correct the acceleration readings based on the tilt angles to remove gravity
  float correctedAccX = filteredAccX - sin(tiltAngleX * PI / 180) * 9.8;
  float correctedAccY = filteredAccY - sin(tiltAngleY * PI / 180) * 9.8;
  float correctedAccZ = filteredAccZ - cos(tiltAngleX * PI / 180) * cos(tiltAngleY * PI / 180) * 9.8;

  // Calculate velocity by integrating corrected acceleration
  velocityX += correctedAccX * dt;
  velocityY += correctedAccY * dt;
  velocityZ += correctedAccZ * dt;

  // Print out results
  Serial.print("Tilt Angle X: ");
  Serial.print(tiltAngleX);
  Serial.print(", Tilt Angle Y: ");
  Serial.print(tiltAngleY);
  Serial.print(", Velocity X: ");
  Serial.print(velocityX);
  Serial.print(", Velocity Y: ");
  Serial.print(velocityY);
  Serial.print(", Velocity Z: ");
  Serial.println(velocityZ);

  // Write data to SD card
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  String dataString = String(millis()) + "," + String(velocityX) + "," + String(velocityY) + "," + String(velocityZ);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  } else {
    Serial.println("error opening datalog.txt");
  }
}
