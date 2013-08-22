#include <FastSerial.h>
#include <math.h>
#include <AP_Common.h>
#include "FHP_Airspeed.h"

//coefficient vectors for polynomial fits to compute airspeed, angle-of-attack, and sideslip
static const double Cq_a[] = {0.9534521504842,-0.00430393572210401,-0.00277037855396079,0.00103120935887708,-0.0468367633310386};
static const double Cq_b[] = {0.952092573306363,-0.00394141318945571,-0.0130476614630522,-0.0111801307500386,-0.0462961679965432};
static const double C_a[] = {0.174765241265372,17.3280853933476,-0.427732349516511,0.8064240343191,0.383140187656122};
static const double C_b[] = {1.88295731457312,17.588305755074,-0.062377037608365,0.570757479525977,-0.321504793057219};
//factor: used to compute airspeed. factor = 133.3226/1.13471 = [conversion factor from Torr to SI]/[assumed air density at 318 ft ASL on a 70 degC day]
static const double factor = 117.494866;

const AP_Param::GroupInfo FHP_Airspeed::var_info[] PROGMEM = {

    // @Param: ENABLE
    // @DisplayName: Airspeed enable
    // @Description: enable airspeed sensor
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("ENABLE",    0, FHP_Airspeed, _enable, 1),

    // @Param: USE
    // @DisplayName: Airspeed use
    // @Description: use airspeed for flight control
    // @Values: 1:Use,0:Don't Use
    AP_GROUPINFO("USE",    1, FHP_Airspeed, _use, 0),

    AP_GROUPEND
};

//constructor
FHP_Airspeed::FHP_Airspeed(FHP* source)
{
	_source = source;
};

void FHP_Airspeed::calibrate(void (*callback)(unsigned long t))
{
	float sum = 0;
    uint8_t c;
    if (!_enable) {
        return;
    }
	//read FHP values
	read();
	//assume that FHP calibration is better than anything we can do on the ground	
}

void FHP_Airspeed::read()
{
	if (!_enable) {
        return;
    }
	
	//read psensor2 from _source
	int i = 0;
	float Qbar, p23, p45, C_beta, C_alpha, Beta_deg=0, Alpha_deg=0, C_q=0, QQ=0;
	
	//read FHP values
	Qbar = -51.72*(_source->fhp_access(2));//P1-P6 in Torr
	p23 = 51.72 *(_source->fhp_access(3));//p2-p3 in Torr
	p45 = 51.72 *(_source->fhp_access(1));//p4 - p5 in torr
	
	//compute sideslip and angle-of-attack coefficients
	C_beta = p23/Qbar;
	C_alpha = p45/Qbar;
	
	//compute angle-of-attack and sideslip
	Beta_deg = 0;
	Alpha_deg = 0;
	for (i = 0;i<5;i++)
	{
		Beta_deg+=C_b[i]*pow(C_beta,float(i));
		Alpha_deg+=C_a[i]*pow(C_alpha,float(i));
	}
	//compute dynamic pressure coefficient
	C_q = 0;
	if(fabs(Beta_deg)>fabs(Alpha_deg))
	{
		for(i = 0;i<5;i++)
		{
			C_q+=Cq_b[i]*pow(C_beta,float(i));
		}
	}
	else
	{
		for(i = 0;i<5;i++)
		{
			C_q+=Cq_a[i]*pow(C_alpha,float(i));
		}
	}
	//compute dynamic pressure
	QQ = C_q/Qbar;
	
	//compute airspeed
	if(QQ>0)
	{
		_airspeed = sqrt(2.*QQ*factor);
	}
	else
	{
		_airspeed = 0;
	}
}

float FHP_Airspeed::get_airspeed()
{
	return _airspeed;
}

float FHP_Airspeed::get_airspeed_cm()
{
	return _airspeed*100;
}

bool FHP_Airspeed::use()
{
	return _enable && _use;
}

bool FHP_Airspeed::enabled()
{
	return _enable;
}

void FHP_Airspeed::set_HIL(float airspeed) {
	_airspeed = airspeed;
}