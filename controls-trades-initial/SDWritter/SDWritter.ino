/*
  SD card datalogger

  The circuit:
   analog sensors on analog ins 0, 1, and 2
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 10
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RH_ASK.h>
#include <Servo.h>
const int MPU = 0x68; // MPU6050 I2C address
float ASens, GSens;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float Uin, Up;
float elapsedTime, currentTime, previousTime;
int c = 0;
int Samples = 0;
int Transmitter = 0;
int RandT, start;

File dataFile;

const int chipSelect = 10;

RH_ASK driver;
Servo servo;

void setup() {

  servo.attach(10);

  // Open serial communications and wait for port to open:
  Serial.begin(19200);
  SPI.begin();
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
 
  if (!driver.init())
  Serial.println("Receiver init failed");

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  dataFile = SD.open("test.txt", FILE_WRITE);
  dataFile.close();

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

  uint8_t buf[1];
  uint8_t buflen = sizeof(buf);
while(Transmitter == 0){
Transmitter = driver.recv(buf, &buflen);
}
}


void loop() {
  switch (Transmitter) {

    case 1:
    analogWrite(10,1);    
    Up = 5;
    servo.write(Up+90);
    break;

    case 10:  
    Up = 5;
    servo.write(Up+90);
    break;

    case 2:
    analogWrite(10,1);
    Up = (180/PI)*sin(millis()/1000);
    servo.write(Up+90);
    break;

    case 20:
    Up = (180/PI)*sin(millis()/1000);
    servo.write(Up+90);
    break;

    case 3:
    analogWrite(10,1);
    if(Up == 5 & (millis()-start)>RandT){
      Up = 0 ;
      servo.write(Up+90);
      RandT = random(2000,4000);
      start = millis();
    }
    if(Up == 0 & (millis()-start)>RandT){
      Up = 5 ;
      servo.write(Up+90);
      RandT = random(2000,4000);
      start = millis();
    }
    break;

    case 30:
    if(Up == 5 & (millis()-start)>RandT){
      Up = 0 ;
      servo.write(Up+90);
      RandT = random(2000,4000);
      start = millis();
    }
    if(Up == 0 & (millis()-start)>RandT){
      Up = 5 ;
      servo.write(Up+90);
      RandT = random(2000,4000);
      start = millis();
    }
    break;

    case 4:
    analogWrite(10,1);
    Up = 0;
    servo.write(Up+90);
    break;

    default:
    Up = 0;
    servo.write(Up+90);
    break;
  }


  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / ASens; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / ASens; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / ASens; // Z-axis value

  // === Read gyroscope data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / GSens; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / GSens;
  GyroZ = (Wire.read() << 8 | Wire.read()) / GSens;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.29)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(0.35)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.13)

  // Read servo inputs
  Uin = analogRead(A0);

  // append sensor data to the string:
    // make a string for assembling the data to log:
  String dataString = String(AccX) + "," + String(AccY) + "," + String(AccZ) + "," + String(GyroX) + "," + String(GyroY) + "," + String(GyroZ) + "," + String(Uin) + "," + String(Up) + "," + String(millis());
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open("test.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening test.txt");
    return;
  }
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 300) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / ASens; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / ASens; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / ASens; // Z-axis value    
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
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}







