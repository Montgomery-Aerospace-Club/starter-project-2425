#include <Wire.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;
MPU6050 mpu;

// Initialize the Kalman filter for each axis
SimpleKalmanFilter kalmanFilterX(1, 1, 0.01);
SimpleKalmanFilter kalmanFilterY(1, 1, 0.01);
SimpleKalmanFilter kalmanFilterZ(1, 1, 0.01);

int16_t ax, ay, az;
int16_t gx, gy, gz;
float accX, accY, accZ;
float accXC, accYC, accZC;                       // Raw acceleration data
float filteredAccX, filteredAccY, filteredAccZ;  // Filtered acceleration data
float accMagnitude;                              // Magnitude of the acceleration vector
float velocity = 0;                              // Current velocity (m/s)
float velocityX, velocityY, velocityZ;
float prevTime = 0;  // Previous time for integration
float dt;            // Time delta for integration

// Lists to store velocity and timestamps
float velocities[1000];
int dataCount = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }

  Serial.println("MPU6050 connected successfully");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1)
      ;
  }
  Serial.println("card initialized.");

  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println("time,vel");
  dataFile.close();



  float tempX, tempY, tempZ;
  for (int i = 0; i < 200; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i >= 50) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      tempX += ax / 16384.0;
      tempY += ay / 16384.0;
      tempZ += az / 16384.0;
    }
  }
  accXC = tempX / 151.0;
  accYC = tempY / 151.0;
  accZC = tempZ / 151.0;

  delay(2000);
  prevTime = millis();
}

void loop() {
  // Read raw acceleration data
  //mpu.getAcceleration(&accX, &accY, &accZ);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw values to g (acceleration in m/s^2)

  accX = (ax / 16384.0 - accXC) * 9.8;
  accY = ((ay / 16384.0) - accYC) * 9.8;
  accZ = ((az / 16384.0) - accZC) * 9.8;

  // Serial.println(accX);
  // Serial.println(accY);
  // Serial.println(accZ);

  if abs ((accX) < 1.5) {
    accX = 0;
  }
  if abs ((accY) < 1.5) {
    accY = 0;
  }
  if abs ((accZ) < 1.5) {
    accZ = 0;
  }

  // Apply Kalman filtering to each axis
  filteredAccX = kalmanFilterX.updateEstimate(accX);
  filteredAccY = kalmanFilterY.updateEstimate(accY);
  filteredAccZ = kalmanFilterZ.updateEstimate(accZ);

  if abs ((filteredAccX) < 1) {
    filteredAccX = 0;
  }
  if abs ((filteredAccY) < 1) {
    filteredAccY = 0;
  }
  if abs ((filteredAccZ) < 1) {
    filteredAccZ = 0;
  }
  // Serial.println(filteredAccX);
  // Serial.println(filteredAccY);
  // Serial.println(filteredAccZ);


  // Get current time and calculate delta time (dt)
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  // Integrate acceleration to compute velocity
  velocityX += filteredAccX * dt * 0.999;
  velocityY += filteredAccY * dt * 0.999;
  velocityZ += filteredAccZ * dt * 0.999;


  velocity = sqrt(sq(velocityX) + sq(velocityY) + sq(velocityZ));
  accMagnitude = sqrt(sq(filteredAccX) + sq(filteredAccY) + sq(filteredAccZ));
  Serial.print("Acceleration: ");
  Serial.print(accMagnitude);
  Serial.print(",Velocity: ");
  Serial.println(velocity);
  if (accMagnitude < 0.5) {
    velocityX = 0;
    velocityY = 0;
    velocityZ = 0;
    velocity = 0;
  }

  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  String dataString = String(millis()) + ",";
  dataString += String(velocity);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  } else {
    Serial.println("error opening datalog.txt");
  }
}
