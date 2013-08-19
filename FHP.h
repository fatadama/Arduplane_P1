//FHP.h - class definition for "five-hole probe" object

#ifndef __FHP_H_INCLUDED__
#define __FHP_H_INCLUDED__

#include <npa700.h>  //pressure sensor library  
#include <HIH6130.h>  //humidity sensor library
#include <I2C.h>

class FHP {
	public:
		FHP();//constructor
		void fhp_read();//read values from sensors
		double fhp_access(int);//return the current value of one pressure
		double fhp_temp_humid(bool);//return the current value of temperature (bool = 0) or humidity (bool = 1)
	private:
		npa700 pSensor0;//pressure sensors
		npa700 pSensor1;
		npa700 pSensor2;
		npa700 pSensor3;
		HIH6130 hSensor;//humidity sensor
		double pressure[4]; //array of size 4 to store all 4 pressure values
		double humidity;  //humidity sensor reading
		double temperature;
};

#endif
