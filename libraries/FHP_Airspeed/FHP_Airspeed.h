//VSCL - class for computing airspeed from five-hole probe readings
//	modelled after the existing AP_Airspeed class for syntactical consistency

#ifndef __FHP_AIRSPEED_INCLUDED__
#define __FHP_AIRSPEED_INCLUDED__

#include <AP_Common.h>
#include <AP_Param.h>
#include <FHP.h>

class FHP_Airspeed {
public:
	//constructor
	FHP_Airspeed(FHP* source);
	void read();//read the source and update airspeed
	void calibrate(void (*callback)(unsigned long t));//calibrate zero airspeed on startup
	void calibrate();//calibrate zero airspeed on startup, overloaded for testing without full APM source code
	double get_airspeed();//return the current airspeed
	int16_t get_airspeed_cm();//return airspeed in cm/s
	double get_alpha();//returns angle-of-attack in centidegrees
	double get_beta();//return sideslip angle in centidegrees
	bool use();//return 1 if airspeed is enabled and use is set to "on"
	bool enabled();//return 1 if airspeed is enabled
	void set_HIL(float airspeed);//set the airspeed from to a given value in HIL mode
	static const struct AP_Param::GroupInfo         var_info[];
private:
	FHP* _source;
	AP_Int8         _use;
    AP_Int8         _enable;
    double           _airspeed;
	double _alpha;
	double _beta;
	//zero-speed calibration values for the pressure sensors.
	double _qbar_offset;
	double _p23_offset;
	double _p45_offset;
};

#endif