/*
    GE NPA700 Device Driver Class for Arduino
    Greatly modified version of Andrew Beckett's version 0.1, by Miguel Alvarez
    
    This class implements the complete function set of the GE NPA700 series of
    digital pressure sensors. 
	
	Changes to Andrew Beckett's original class
	-Changed from Wayne Truchsess I2C library to the default Wire.h Arduino library
	-Fixed the write address function which previously wasnt working.
	-There is no longer returning of error states
*/

#ifndef __NPA700_H__
#define __NPA700_H__

#include <Arduino.h>
#include <I2C.h>

class npa700 {

public:
    npa700(); // default constructor
    npa700(int i2cAddress); // constructor with non-default address
    
    void read(); // read the sensor, return error state
    double getPressure(); // return the last pressure reading in ADC counts (14 bit)
    double getTemperature(); // return the last temperature reading in ADC counts (11 bit)
    uint8_t getStatus(); // return the 2-bit status flag (0x00 for a healthy sensor)
	void changeScale(double);//change the full scale value of the sensor.
    //void changeAddress(int newAddr); // assign a sensor a new 7-bit I2C address
    // drasnor: YOU HAD BETTER ENSURE newAddress CONTAINS A VALID 7-BIT ADDRESS
    //          BECAUSE IF IT DOESN'T YOU WILL KILL THE SENSOR!!!

private:
	int thisAddress; // NPA700 7-bit slave I2C address (default 0x28)
    uint16_t pressure; // 14-bit pressure reading from sensor internal ADC
    uint16_t temperature; // 11-bit temperature reading from sensor internal ADC
	double temperatureC;
	double pressurePSI;
    uint8_t status; // 2-bit status flag from sensor
    uint8_t dataBuf[4]; // dedicated sensor input I2C buffer
	double pmin;//maximum gauge pressure. differential pressure return assumed
};

#endif