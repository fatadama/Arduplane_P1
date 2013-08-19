//FHP.cpp - function definitions for five hole probe object

#include<I2C.h>
#include "FHP.h"

FHP::FHP()
{
//initialize pressure sensor addresses
pSensor0 = npa700(0x30);
pSensor1 = npa700(0x31);
pSensor2 = npa700(0x32);
pSensor3 = npa700(0x33);
//temperature/humidity sensor address
hSensor = HIH6130(0x27);	
//initialize readings to zero
	humidity = 0;
	temperature = 0;
//initialize pmin to the default,
}

void FHP::fhp_read()
{
//read values from all sensors:
	pSensor0.read();
	pSensor1.read();
	pSensor2.read();
	pSensor3.read();
	hSensor.read();
}

double FHP::fhp_access(int i)
{
	//access the values of one of the pressure sensors
	switch (i)
	{
		case 0:
			pressure[0] = pSensor0.getPressure();
			return pressure[0];
			break;
		case 1:
			pressure[1] = pSensor1.getPressure();
			return pressure[1];
			break;
		case 2:
			pressure[2] = pSensor2.getPressure();
			return pressure[2];
			break;
		case 3:
			pressure[3] = pSensor3.getPressure();
			return pressure[3];
			break;
	}
}

double FHP::fhp_temp_humid(bool flag)
{
//if zero, return temperature. else, return humidity.
	if (!flag)
	{
		//return temp
		temperature = hSensor.getTemperature();
		return temperature;
	}
	else
	{
		humidity = hSensor.getHumidity();
		return humidity;
	}
}
