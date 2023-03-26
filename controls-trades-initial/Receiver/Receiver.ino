// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h> 
 
// Create Amplitude Shift Keying Object
RH_ASK rf_driver;
int Transmit = 1;

void setup()
{
    // Initialize ASK Object
    rf_driver.init();
    // Setup Serial Monitor
    Serial.begin(19200);

    
}
 
void loop()
{
    // Set buffer to size of expected message
    uint8_t Send;
    uint8_t Receive = Send;
    uint8_t Sendlen = sizeof(Send);
    // Check if received packet is correct size
    if (rf_driver.recv((uint8_t*)&Send, &Sendlen)&& Sendlen == sizeof(Send))
    {
      // Message received with valid checksum
      Serial.println(Receive);    
      delay(2);   
    }    
}