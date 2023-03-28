#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>
//------------------------------------------------------------------------------
MPU6050 mpu;

// setup AVR I2C
void userSetup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
  mpu.setSleepEnabled(false);
}
//-----------------------------------------------------------------------------

//const int MPU = 0x68; // MPU6050 I2C address
float ASens, GSens;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
int16_t ax,ay,az,gx,gy,gz;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime;
long  previousTime;
int RandT, start,time,runtime;
float theta,Uc,Up;


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;



void setup() {    
  servo1.attach(10);
  servo2.attach(11);
  servo3.attach(12);
  servo4.attach(13);

  userSetup();
  // Open serial communications and wait for port to open:
  
  Serial.begin(4800);
  
  ASens = 16384;
  GSens = 131;

  // Call this function if you need to get the IMU error values for your module
  // mpu.setDMPEnabled(false);
  calculate_IMU_error();
  delay(600000);
  previousTime = millis();
}

void loop() {
runtime = millis();
while(runtime < 120000){
    Control();
    Up = 10;
    servo4.write(Up+90);
    
}
while(runtime < 240000){
    Control();
    Up = 0;
    servo4.write(Up+90);
    
}

// start = millis();
// while(millis() < 360000){
//   Control();
//   if(Up == 10 && ((millis()-start) > RandT)){
//       Up = 0 ;
//       servo4.write(Up+90);
    
//       RandT = random(4000,6000);
//       start = millis();
//     }
//   if(Up == 0 && ((millis()-start)) > RandT){
//       Up = 10 ;
//       servo4.write(Up+90);

//       RandT = random(4000,6000);
//       start = millis();
//     }
// }
}

// void loop(){
//   Control();
//   motion_print();
// }

void motion_print(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  AccX = ax / ASens; // X-axis value
  AccY = ay / ASens; // Y-axis value
  AccZ = az / ASens; // Z-axis value  

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan2(AccY , sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(1.55) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan2(AccX , sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.14)  
  Serial.println(accAngleX);
}

void Control(){
  float K1 = 1.000;
  float K2 = 1.597;
  
  int i;
  for(i=0; i <= 5; i++){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  
    
    AccX = AccX + ax / ASens; // X-axis value
    AccY = AccY + ay / ASens; // Y-axis value
    AccZ = AccZ + az / ASens; // Z-axis value  

    // Correct the outputs with the calculated error values
    GyroX = GyroX + gx/GSens - GyroErrorX; // GyroErrorX ~(-5.92)
    GyroY = GyroY + gy/GSens - GyroErrorY; // GyroErrorY ~(1.05)
    GyroZ = GyroZ + gz/GSens - GyroErrorZ; // GyroErrorZ ~ (-1.17)
  }
  AccX = AccX / i; // X-axis value
  AccY = AccY / i; // Y-axis value
  AccZ = AccZ / i; // Z-axis value  

  // Correct the outputs with the calculated error values
  GyroX = GyroX / i; // GyroErrorX ~(-5.92)
  GyroY = GyroY / i; // GyroErrorY ~(1.05)
  GyroZ = GyroZ / i; // GyroErrorZ ~ (-1.17)

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan2(AccY , sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(1.55) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan2(AccX , sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.14)
  // === Read gyroscope data === //

  elapsedTime = (millis() - previousTime) / 1000; // Divide by 1000 to get seconds
  previousTime = millis();

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;// deg/s * s = deg

  // Complementary filter - combine acceleromter and gyro angle values
  theta = 0.96 * gyroAngleY + 0.04 * accAngleY;

 // === Calculate desired servo value ===//
  Uc = -K1*theta-K2*(GyroY);

  if (abs(Uc) > 15) {
    Uc = (abs(Uc)/Uc)*15;
  }

  servo1.write(Uc+90);
  servo2.write(Uc+90);
  servo3.write(Uc+90);

}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 300 times
  int c = 0;

  while (c < 300) {

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    AccX = ax / ASens; // X-axis value
    AccY = ay / ASens; // Y-axis value
    AccZ = az / ASens; // Z-axis value  

    // Sum all readings
    AccErrorX = AccErrorX + ((atan2(AccY , sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan2(AccX , sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));  

    // Sum all readings
    GyroErrorX = GyroErrorX + (gx / GSens);
    GyroErrorY = GyroErrorY + (gy / GSens);
    GyroErrorZ = GyroErrorZ + (gz / GSens);
    c++;
  }
  //}
  AccErrorX = AccErrorX / c;
  AccErrorY = AccErrorY / c;
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