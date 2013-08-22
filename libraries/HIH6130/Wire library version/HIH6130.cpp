/*
    Honeywell Humidlcon HIH6130 Class for Arduino
     by Miguel Alvarez, 6/2013
    
	This is a basic library for the HIH6130 humidity/temperature sensor which 
	implements I2C reading functions and converts the data into proper units
	
*/

#include "HIH6130.h"
#include <Arduino.h>
#include <Wire.h>

//Default constructor
HIH6130::HIH6130() {
    thisAddress = 0x27; // default I2C sensor address
}

// constructor with non-default address
HIH6130::HIH6130(int i2cAddress) {
    thisAddress = i2cAddress;
}

// Public Methods
// --------------

// read the sensor (4-byte mode)
void HIH6130::read() {
	//This tells the sensor to update its readings
	Wire.beginTransmission(thisAddress);
	Wire.endTransmission();
	
	//Now that readings are updated, we will request the 4 byte data
	Wire.requestFrom(thisAddress, 4);    
	int i=0;
	while(Wire.available())    
	{ 
		dataBuf[i]=Wire.read(); //fill the data buffer with the 4 data bytes
		 i++;
	}
	//separate the bits into the corresponding data
	status = (dataBuf[0] & 0xC0) >> 6; // status is two most MSb of first byte
	humidity = (dataBuf[0] & 0x3F) << 8; // humidity MSB less the status bits
	humidity += dataBuf[1]; // pressure LSB is the entire second byte
	temperature = dataBuf[2] << 6; // most significant 8 bits in the 14-bit temperature read
	temperature += dataBuf[3] >> 2; // least significant 6 bits in the 14-bit temperature read
}
// return the last humidity reading in %RH
double HIH6130::getHumidity() {
	humidityRH=(humidity/(pow(2,14)-2))*100;
    return humidityRH;
}

// return the last temperature reading in degrees Celcius
double HIH6130::getTemperature() {
	temperatureC=(temperature/(pow(2,14)-2))*165-40;
    return temperatureC;
}

// return the 2-bit status flag
uint8_t HIH6130::getStatus() {
    return status;
}