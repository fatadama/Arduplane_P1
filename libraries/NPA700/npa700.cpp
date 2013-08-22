
/*
    GE NPA700 Device Driver Class for Arduino
	TDW added back support for ArduPilot Mega I2C library for integration with APM
	address changing is NOT currently supported
	use the Wire library version for accomplishing address changes to I2C pressure sensors
		twoodbury@tamu.edu
	
    Greatly modified version of Andrew Beckett's version 0.1, by Miguel Alvarez
    
    This class implements the complete function set of the GE NPA700 series of
    digital pressure sensors. 
	
	Changes to Andrew Beckett's original class
	-Changed from Wayne Truchsess I2C library to the default Wire.h Arduino library
	-Fixed the write address function which previously wasnt working.
	-There is no longer returning of error states
*/


#include "npa700.h"
#include <Arduino.h>
#include <I2C.h>

// Constructors
// ------------

// default constructor
npa700::npa700() {
    thisAddress = 0x28; // default I2C sensor address
	pmin = .36259;//The full scale range is assumed to be +/- pmin
}

// constructor with non-default address
npa700::npa700(int i2cAddress) {
    thisAddress = i2cAddress;
	pmin = .36259;//The full scale range is assumed to be +/- pmin
}

// Public Methods
// --------------

void npa700::changeScale(double value)
{
	//reset the full-scale range
	pmin = value;
}

// read the sensor (4-byte mode)
void npa700::read() {
		uint8_t  error = 0;
		error |= I2c.read(thisAddress,4,dataBuf);
		if (!error)
		{
			status = (dataBuf[0] & 0xC0) >> 6; // status is two most MSb of first byte
			pressure = (dataBuf[0] & 0x3F) << 8; // pressure MSB less the status bits
			pressure += dataBuf[1]; // pressure LSB
			temperature = dataBuf[2] << 3; // most significant 8 bits in the 11-bit temperature read
			temperature += dataBuf[3] >> 5; // least significant 3 bits in the 11-bit temperature read
		}
}

// return the last raw pressure reading
double npa700::getPressure() {
	pressurePSI= -pmin+double(pressure-1638)/(14745-1638)*(2*pmin);
    return pressurePSI;
}

// return the last temperature reading converted into degrees C
double npa700::getTemperature() {
	temperatureC=(temperature*200)/pow(2,11)-50;
    return temperatureC;
}

// return the 2-bit status flag
uint8_t npa700::getStatus() {
    return status;
}
/*
void npa700::changeAddress(int newAddr){
  byte command[3] = {0xA0, 0x00, 0x00}; //command to put sensor in command mode
  byte cmdRead[3]= {0x02, 0x00, 0x00};	//command to read EEPROM word 2 from sensor
  byte bufRead[3];						// buffer to store EEPROM word 2 first byte should be 5A
  byte bufWrite[3]={0x42, 0x00, 0x00};	// Write data to send with new address first byte is 0x42 and other  2 bytes will be the modified word 2 with new address
  byte exitCmd[3]= {0x80, 0x00, 0x00};	//exit command after writing address, if address isnt written properly it will break the board
  int readAddr;							//int to store address read from word 2 to confirm it matches the expected address

  //begin command mode transmission
   I2c.beginTransmission(thisAddress); 
  for(int i=0; i<3; i++){
    Wire.write(command[i]);        
  }
  Wire.endTransmission();
  
  //begin EEPROM word 2 request transmission
  Wire.beginTransmission(thisAddress); 
  for(int i=0; i<3; i++){
    Wire.write(cmdRead[i]);        
  }
  Wire.endTransmission();   
  
  //now request the EEPROM word 2 data. The first byte should be 0x5A. The next two bytes are word 2. byte 1 is bits 15:9 and byte 2 is bits 7:0
  //The I2C address is held in 7 bits [9:3]
  Wire.requestFrom(thisAddress, 3);
  int i=0;
  while(Wire.available())    
  { 
    bufRead[i] = Wire.read(); 
    i++;
  }
  
  Serial.println("Buffer contents");
  Serial.println(bufRead[0],HEX);
  Serial.println(bufRead[1],BIN);
  Serial.println(bufRead[2],BIN);
  
  readAddr=(bufRead[2] & 0xF8) >> 3; // least significant 5 bits of the I2C address
  readAddr+= (bufRead[1] & 0x03) << 5; // most significant 2 bits of the I2C address
  
  //bits [9:3] will be put into an address we can read to verify it matches the current address of the board
  Serial.println(readAddr,HEX);
  
  //when we confirm that byte 0 is 0x5A and the address in bits [9:3] match the sensors current address, we can continue to write the new address
  if(bufRead[0]==0x5A && readAddr==thisAddress){
    Serial.println("matched");
	
	//modify bits from word 2 to write to the EEPROM
    bufWrite[2] = bufRead[2] & 0x07; // copy the parts of the buffer that aren't address
    bufWrite[1] = bufRead[1] & 0xFC; // for both bytes
    bufWrite[2] += newAddr << 3; // copy in the least significant 5 bits to LSB
    bufWrite[1] += newAddr >> 5; // copy in the most significant 2 bits to the MSB
    Serial.println("BufferWrite contents");
    Serial.println(bufWrite[1],BIN);
    Serial.println(bufWrite[2],BIN);
    
	//Begin transmission to write the new version of Word 2 with the changed address bits
    Wire.beginTransmission(thisAddress);
    for(int i=0; i<3; i++){
      Wire.write(bufWrite[i]);      
    }
    Wire.endTransmission();
    
	//After writing the new Word 2, we will write the exit command still using the current address. Once exited, the address is now the new one
    Wire.beginTransmission(thisAddress);
    for(int i=0; i<3; i++){
      Wire.write(exitCmd[i]);      
    }
    Wire.endTransmission();
	
	//confirmation message that address was written
    Serial.print("Address ");
    Serial.print(newAddr,HEX);
    Serial.println(" written ");
    
  }
  else{
    Serial.println("Error, buffer contents dont match expected values");
	Serial.println("Please unplug the Vcc connection (5V), press the reset button on the arduino, and try again");
  }
  delay(100);
}
*/