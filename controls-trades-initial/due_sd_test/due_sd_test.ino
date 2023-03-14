#include <SD.h>
#include <SPI.h>
char fileName[] = "test1.txt";
const int chipSelect = 10;
const int accel_pin = 3;
const int angular_vel_pin = 4;
const int num_lines = 100;
int written = 0;
double accel = 0;
double angular_vel = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Simple SD Card Demo");

   if (SD.begin(chipSelect))
    {
    Serial.println("SD card is present & ready");
    } 
    else
    {
    Serial.println("SD card missing or failure");
    while(1);  //wait here forever
    }
}

void loop() {
  // Read from pin
  while(written < num_lines){
    accel = digitalRead(3);
    angular_vel = digitalRead(4);
    writeToFile(accel, angular_vel);
    ++written;
  }
  
}

void writeToFile(double accel, double angular_vel)
{
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) // it opened OK
    {
    Serial.println("Writing to test1.txt");
    myFile.println("%lf ", accel);
    myFile.println("%lf \n", angular_vel);
    myFile.close(); 
    Serial.println("Done");
    }
  else 
    Serial.println("Error opening simple.txt");
}
