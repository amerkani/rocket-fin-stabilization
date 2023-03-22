#include <Wire.h>
#include <Servo.h>
const int MPU = 0x68; // MPU6050 I2C address
float ASens, GSens;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ, GyroOld, GyroSum;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime;
long  previousTime;
float K1, K2;
float theta,u;
int c = 0;

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  K1 = 1.000;
  K2 = 1.597;
    
  servo1.attach(10);
  servo2.attach(11);
  servo3.attach(12);

  // Open serial communications and wait for port to open:
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x01);                  //Set the register bits as 00010000 (+/- 4g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x01);                   // Set the register bits as 00010000 (500deg/s full scale)
  Wire.endTransmission(true);
  ASens = 8192.0;
  GSens = 65.5;

  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(5);
  previousTime = millis();
}

void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / ASens; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / ASens; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / ASens; // Z-axis value    

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan2(AccY , sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(1.55) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan2(AccX , sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.14)
  // === Read gyroscope data === //

  elapsedTime = (millis() - previousTime) / 1000; // Divide by 1000 to get seconds
  previousTime = millis();

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / GSens; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / GSens;
  GyroZ = (Wire.read() << 8 | Wire.read()) / GSens;

  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-5.92)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(1.05)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-1.17)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;// deg/s * s = deg

  // Complementary filter - combine acceleromter and gyro angle values
  theta = 0.96 * gyroAngleY + 0.04 * accAngleY;

 // === Calculate desired servo value ===//
  u = -K1*theta-K2*(GyroY);

  analogWrite(44,u);
  
  if (abs(u) > 15) {
    u = (abs(u)/u)*15;
  }

  servo1.write(u+90);
  servo2.write(u+90);
  servo3.write(u+90);

}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 300 times
  while (c < 300) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / ASens ;
    AccY = (Wire.read() << 8 | Wire.read()) / ASens ;
    AccZ = (Wire.read() << 8 | Wire.read()) / ASens ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan2(AccY , sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan2(AccX , sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 300 to get the error value
  AccErrorX = AccErrorX / c;
  AccErrorY = AccErrorY / c;
  c = 0;
  // Read gyro values 300 times
  while (c < 300) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / GSens);
    GyroErrorY = GyroErrorY + (GyroY / GSens);
    GyroErrorZ = GyroErrorZ + (GyroZ / GSens);
    c++;
  }
  //Divide the sum by 300 to get the error value
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;
  // Print the error values on the Serial Monitor
  Serial.print(F("AccErrorX: "));
  Serial.println(AccErrorX);
  Serial.print(F("AccErrorY: "));
  Serial.println(AccErrorY);
  Serial.print(F("GyroErrorX: "));
  Serial.println(GyroErrorX);
  Serial.print(F("GyroErrorY: "));
  Serial.println(GyroErrorY);
  Serial.print(F("GyroErrorZ: "));
  Serial.println(GyroErrorZ);
}