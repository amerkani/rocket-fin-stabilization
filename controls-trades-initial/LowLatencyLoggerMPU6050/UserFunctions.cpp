// User data functions.  Modify these functions for your data items.
#include "UserTypes.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
//------------------------------------------------------------------------------
MPU6050 mpu;
static uint32_t startMicros;
// Acquire a data record.
void acquireData(data_t* data, int Utot) {
  data->time = micros();
  mpu.getMotion6(&data->ax, &data->ay, &data->az,
                 &data->gx, &data->gy, &data->gz);
  data->uc = ((Utot / 1000U) % 10)*10+((Utot / 100U) % 10);    //Controller Fins Servo Position
  data->up = ((Utot / 10U) % 10)*10+((Utot / 1U) % 10);    //Perturbation Fin Servo Position
}

// setup AVR I2C
void userSetup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
}

// Print a data record.
void printData(Print* pr, data_t* data) {
  if (startMicros == 0) {
    startMicros = data->time;
  }
  pr->print(data->time- startMicros);
  pr->write(',');
  pr->print(data->ax);
  pr->write(',');
  pr->print(data->ay);
  pr->write(',');
  pr->print(data->az);
  pr->write(',');
  pr->print(data->gx);
  pr->write(',');
  pr->print(data->gy);
  pr->write(',');
  pr->print(data->gz);
  pr->write(',');
  pr->print(data->uc);
  pr->write(',');
  pr->println(data->up);
}

// Print data header.
void printHeader(Print* pr) {
  startMicros = 0;
  pr->println(F("micros,ax,ay,az,gx,gy,gz,uc,up"));
}