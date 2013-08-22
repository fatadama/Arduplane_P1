/*
    GE NPA700 Device Driver Class for Arduino
    version 0.1 by Andrew Beckett, 11/2012
    
    This class implements the complete function set of the GE NPA700 series of
    digital pressure sensors. This class relies on the enhanced I2C class written
    by Wayne Truchsess. 
*/

#ifndef __NPA700_H__
#define __NPA700_H__

#include <Arduino.h>
#include <I2C.h>

class npa700 {

public:
    npa700(); // default constructor
    npa700(uint8_t i2cAddress); // constructor with non-default address
    
    uint8_t read(); // read the sensor, return error state
    uint16_t getPressure(); // return the last pressure reading in ADC counts (14 bit)
    uint16_t getTemperature(); // return the last temperature reading in ADC counts (11 bit)
    uint8_t getStatus(); // return the 2-bit status flag (0x00 for a healthy sensor)
    uint8_t writeAddress(uint8_t newAddress); // assign a sensor a new 7-bit I2C address, return error state
    // drasnor: YOU HAD BETTER ENSURE newAddress CONTAINS A VALID 7-BIT ADDRESS
    //          BECAUSE IF IT DOESN'T YOU WILL KILL THE SENSOR!!!

private:
    uint8_t thisAddress; // NPA700 7-bit slave I2C address (default 0x28)
    uint16_t pressure; // 14-bit pressure reading from sensor internal ADC
    uint16_t temperature; // 11-bit temperature reading from sensor internal ADC
    uint8_t status; // 2-bit status flag from sensor
    uint8_t dataBuf[3]; // dedicated sensor input I2C buffer
};

#endif