/*
    GE NPA700 Device Driver Class for Arduino
    version 0.1 by Andrew Beckett, 11/2012
    
    This class implements the complete function set of the GE NPA700 series of
    digital pressure sensors. This class relies on the enhanced I2C class written
    by Wayne Truchsess. 
*/

#include "npa700.h"
#include <Arduino.h>
#include <I2C.h>

// Constructors
// ------------

// default constructor
npa700::npa700() {
    thisAddress = 0x28; // default I2C sensor address
}

// constructor with non-default address
npa700::npa700(uint8_t i2cAddress) {
    thisAddress = i2cAddress;
}

// Public Methods
// --------------

// read the sensor (4-byte mode), return error state
uint8_t npa700::read() {
    uint8_t error = 0; // error state
    error |= I2c.read(thisAddress,4,dataBuf); // read the sensor contents
    // if no read error, unpack the packet
    if(!error) {
        status = (dataBuf[0] & 0xC0) >> 6; // status is two most MSb of first byte
        pressure = (dataBuf[0] & 0x3F) << 8; // pressure MSB less the status bits
        pressure += dataBuf[1]; // pressure LSB
        temperature = dataBuf[2] << 3; // most significant 8 bits in the 11-bit temperature read
        temperature += dataBuf[3] >> 5; // least significant 3 bits in the 11-bit temperature read
    }
    return error; // hopefully 0, otherwise lookup specific error code in AVR docs.
}

// return the last pressure reading in ADC counts (14 bit)
uint16_t npa700::getPressure() {
    return pressure;
}

// return the last temperature reading in ADC counts (11 bit)
uint16_t npa700::getTemperature() {
    return temperature;
}

// return the 2-bit status flag
uint8_t npa700::getStatus() {
    return status;
}

// assign a sensor a new I2C address, return error state; must be issued within 6 ms of power up.
// ***************************************************************************************************
// drasnor: this method is super-dangerous, please don't try to use it until after you get read to
// work. Do read NPAApplicationGuide.pdf pages 10-12. If you encounter an error here you will need
// to modify this with debugging output to try and catch the error. It's designed to bail out and not
// issue an actionExitCommandMode if there's an error at any point though I have no idea if it'll work
// after actionWriteEEPROMword02 is issued. If EEPROM word 02 is overwritten with garbage, the sensor
// is toast.
// ***************************************************************************************************
uint8_t npa700::writeAddress(uint8_t newAddress) {
    uint8_t error = 0; // error state
    
    uint8_t actionEnterCommandMode = 0xA0; // action to enter command mode
    uint8_t actionReadEEPROMword02 = 0x02; // action to read EEPROM word 02
    uint8_t actionWriteEEPROMword02 = 0x42; // action to write EEPROM word 02
    uint8_t actionExitCommandMode = 0x80; // action to exit command mode
    uint8_t confirmEEPROMword02Read = 0x5A; // response byte for EEPROM fetch
    
    uint8_t inBuffer[4]; // input buffer
    uint8_t outBuffer[2]; // output word buffer
    uint8_t i2cAddress; // temporary holder for i2c 7-bit addresses
    
    error |= I2c.write(thisAddress,actionEnterCommandMode); // enter command mode
    // first abort: enter command mode write error
    if(!error) error |= I2c.write(thisAddress,actionReadEEPROMword02); // queue EEPROM word 02 read
    // second abort: action to read EEPROM word 02 error
    if(!error) error |= I2c.read(thisAddress,3,inBuffer); // read EEPROM word 02
	
	
	Serial.println("buffer contents");
	Serial.println(inBuffer[0],HEX);
	Serial.println(inBuffer[1],HEX);
	Serial.println(inBuffer[2],HEX);
	Serial.println(inBuffer[3],HEX);
	
    // third abort: read error for EEPROM word 02
    if(!error) {
        // if the buffer contents don't match
        if(inBuffer[1] != confirmEEPROMword02Read) {
			error = 0xEF; //effffff (bail out)
			Serial.println("buffer contents dont match");
			}
			
        else { // otherwise retrieve the current I2C address
			Serial.println("Buffer Contents Match");
            i2cAddress = (inBuffer[2] & 0xF8) >> 3; // least significant 5 bits of the I2C address
            i2cAddress += (inBuffer[1] & 0x03) << 5; // most significant 2 bits of the I2C address
            // if the current I2C address doesn't match the actual address
            if(i2cAddress != thisAddress) error = 0xEF; //effffff (bail out)
            else { // otherwise update the contents of EEPROM word 02 with the new address
				Serial.println("Writing new address");

                outBuffer[2] = inBuffer[2] & 0x07; // copy the parts of the buffer that aren't address
                outBuffer[1] = inBuffer[1] & 0xFC; // for both bytes
                outBuffer[2] += newAddress << 3; // copy in the least significant 5 bits to LSB
                outBuffer[1] += newAddress >> 5; // copy in the most significant 2 bits to the MSB
                error |= I2c.write(thisAddress,actionWriteEEPROMword02,outBuffer,2); // write the new EEPROM word 02
            }
        }
    }
    // fourth abort: bad packet, bad i2c address, or botched write
    if(!error) {
		Serial.println("ActionExit");
		error |= I2c.write(thisAddress,actionExitCommandMode); // exit command mode
	}
	Serial.println(newAddress,HEX);
    thisAddress = newAddress; // set the new address for future use
	//Serial.println(thisAddress);
    return error;
}

// Private Methods
// ---------------
