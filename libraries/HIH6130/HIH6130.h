/*
    Honeywell Humidlcon HIH6130 Class for Arduino
     by Miguel Alvarez, 6/2013
    
	This is a basic library for the HIH6130 humidity/temperature sensor which 
	implements I2C reading functions and converts the data into proper units
	
*/

#ifndef __HIH6130_H_INCLUDED__
#define __HIH6130_H_INCLUDED__

#include <Arduino.h>
#include <I2c.h>

class HIH6130 {

public:
    HIH6130(); // default constructor
    HIH6130(int i2cAddress); // constructor with non-default address
    
    void read(); // read the sensor (4-byte mode)
    double getHumidity(); // return the last humidity reading in %RH
    double getTemperature(); // return the last temperature reading in degrees Celcius
    uint8_t getStatus(); // return the 2-bit status flag (0x00 for a healthy sensor)
    

private:
    int thisAddress; // NPA700 7-bit slave I2C address (default 0x28)
    double humidityRH;	//converted humidity value into a %RH
	double temperatureC;	//converted temperature reading into degrees C
	uint16_t humidity; // 14-bit pressure reading from sensor internal ADC
    uint16_t temperature; // 11-bit temperature reading from sensor internal ADC
    uint8_t status; // 2-bit status flag from sensor
    uint8_t dataBuf[4]; // dedicated sensor input I2C buffer
};

#endif