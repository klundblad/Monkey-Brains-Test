/*
 * Selects Channel V0A on the AD7616 to stream data from
 * and performs conversions indefinitely from the Channel Select
 * register on the device
 */
#include "Arduino.h"
#include "SerialPort.h"

SerialPortClass test;

/* Defining read and write placeholders */
uint16_t readVar; // arbitrary value for read method stored 
uint16_t writeVar; // arbitrary value for write method stored 
int setChannel; // arbitrary value for setting channel method stored
int readChanReg; // arbitrary value for reading channel method stored
int convert; // arbitrary value for conversion method stored

/*
 * Initialization of the chip is performed, pins are set to their
 * inactive states, pins are specified as either inputs or outputs, 
 * serial communication starts
 */
void setup() {
  Serial.begin(115200); 
  ad7616_reset(); 
  test.init(); 
  test.reset(); 
  /* Registers are now default state */ 
}

/*
 * Waits for a signal be fed to serial to begin 
 * reading from the serial channel and streaming data
 * via a "dummy conversion" and two repeated conversions in  
 * an indefinite loop
 */
void loop() {
  while (!Serial.available()) {
    
  }
  
  /* Setting Channel to VA0 */
  setChannel = ad7616_set_channel(AD7616_VA0); 

  /* Reads the channel register using a pointer to the read variable */
  readChanReg = ad7616_read(AD7616_REG_CHANNEL, &readVar);
  
  /* Conversion to stream data is performed for Channel A data */
  convert = ad7616_conversion(&readVarA, &readVarB); 

  /* Performs two conversions indefinitely */
  while (1) {
    convert = ad7616_conversion(&readVarA, &readVarB);
    Serial.println(readVarA);
    convert = ad7616_conversion(&readVarA, &readVarB);
    Serial.println(readVarA);
  }
}
