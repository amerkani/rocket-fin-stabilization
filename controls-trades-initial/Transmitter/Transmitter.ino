// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h> 
 
// Create Amplitude Shift Keying Object
RH_ASK rf_driver;

String inString = "";  // string to hold input
int Send = 404; 

void setup()
{
  // Initialize ASK Object
  rf_driver.init();
  Serial.begin(19200);
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);

  Serial.println();
  Serial.println(F("type:"));
  Serial.println(F("1 - Const Pert w/ Control "));
  Serial.println(F("10 - Const Pert w/ NO Control "));
  Serial.println(F("2 - Periodic Pert w/ Control "));
  Serial.println(F("20 - Periodic Pert w/ NO Control "));
  Serial.println(F("3 - Pseudo Rand Pert w/ Control "));
  Serial.println(F("30 - Pseudo Rand Pert w/  NO Control "));
  Serial.println(F("4 - NO Pert w/ Control "));
  Serial.println(F("All Else - NO Pert w/ NO Control "));  
  Serial.println(F("123 - convert file to csv"));
}
 
void loop()
{
  // Read serial input:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      Send = inString.toInt();
      // clear the string for new input:
      inString = "";
      Serial.println(Send);
    }
  }

  rf_driver.send((uint8_t *)&Send, sizeof(Send));
  rf_driver.waitPacketSent();
}