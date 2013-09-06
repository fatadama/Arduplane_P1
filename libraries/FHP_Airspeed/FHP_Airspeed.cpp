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
	_qbar_offset = 0;
	_p23_offset = 0;
	_p45_offset = 0;
};

void FHP_Airspeed::calibrate(void (*callback)(unsigned long t))
{
    uint16_t c;
    if (!_enable) {
        return;
    }
	
	double Qbar=0,p23=0,p45=0;
	int32_t offset1 = 0,offset2 = 0,offset3 = 0;
	
	for(c = 0;c<1000;c++)
	{
		_source->fhp_read();
		//read pressure sensor values
		Qbar = _source->fhp_access(2);//P1-P6 in Torr
		p23 = _source->fhp_access(3);//p2-p3 in Torr
		p45 = _source->fhp_access(1);//p4 - p5 in torr
		//add to values for calibration
		offset1+=1e5*Qbar;
		offset2+=1e5*p23;
		offset3+=1e5*p45;
	}
	offset1/=c;
	offset2/=c;
	offset3/=c;
	//set the offsets IN PSI
	_qbar_offset = 1e-5*offset1;
	_p23_offset = 1e-5*offset2;
	_p45_offset = 1e-5*offset3;
}

void FHP_Airspeed::calibrate()
{
//calibrate the differential pressure sensors to zero offset
    uint16_t c;
    if (!_enable) {
        //return;
    }
	
	double Qbar=0,p23=0,p45=0;
	int32_t offset1 = 0,offset2 = 0,offset3 = 0;
	
	for(c = 0;c<1000;c++)
	{
		_source->fhp_read();
		//read pressure sensor values
		Qbar = _source->fhp_access(2);//P1-P6 in psi
		p23 = _source->fhp_access(3);//p2-p3 in psi
		p45 = _source->fhp_access(1);//p4 - p5 in psi
		//add to values for calibration
		offset1+=1e5*Qbar;
		offset2+=1e5*p23;
		offset3+=1e5*p45;
	}
	//average values
	offset1/=c;
	offset2/=c;
	offset3/=c;
	//set the offsets IN PSI
	_qbar_offset = 1e-5*offset1;
	_p23_offset = 1e-5*offset2;
	_p45_offset = 1e-5*offset3;
}

void FHP_Airspeed::read()
{
	if (!_enable) {
        //return;
    }
	
	//read psensor2 from _source
	int i = 0;
	double Qbar, p23, p45, C_beta, C_alpha, Beta_deg=0, Alpha_deg=0, C_q=0, QQ=0;
	
	//read FHP values
	Qbar = -double(51.72)*(_source->fhp_access(2)-_qbar_offset);//P1-P6 in Torr
	p23 = double(51.72) *(_source->fhp_access(3)-_p23_offset);//p2-p3 in Torr
	p45 = double(51.72) *(_source->fhp_access(1)-_p45_offset);//p4 - p5 in torr
	
	//compute sideslip and angle-of-attack coefficients
	C_beta = p23/Qbar;
	C_alpha = p45/Qbar;
	
	//compute angle-of-attack and sideslip
	Beta_deg = 0;
	Alpha_deg = 0;
	Beta_deg+=C_b[0]+C_b[1]*C_beta;
	Alpha_deg+=C_a[0]+C_a[1]*C_alpha;
	for (i = 2;i<5;i++)
	{
		Beta_deg+=C_b[i]*pow(C_beta,1.0*i);
		Alpha_deg+=C_a[i]*pow(C_alpha,1.0*i);
	}
	//compute dynamic pressure coefficient
	C_q = 0;
	if(fabs(Beta_deg)>fabs(Alpha_deg))
	{
		C_q+=Cq_b[0]+Cq_b[1]*C_beta;
		for(i = 2;i<5;i++)
		{
			C_q+=Cq_b[i]*pow(C_beta,1.0*i);
		}
	}
	else
	{
		C_q+=Cq_a[0]+Cq_a[1]*C_alpha;
		for(i = 2;i<5;i++)
		{
			C_q+=Cq_a[i]*pow(C_alpha,1.0*i);
		}
	}
	
	//compute dynamic pressure
	QQ = C_q/Qbar;
	
	//store angles
	_alpha = Alpha_deg;
	_beta = Beta_deg;
	
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

double FHP_Airspeed::get_airspeed()
{
	return _airspeed;
}

double FHP_Airspeed::get_alpha()
{
	return _alpha;
}

double FHP_Airspeed::get_beta()
{
	return _beta;
}

int16_t FHP_Airspeed::get_airspeed_cm()
{
	return int16_t(_airspeed*100);
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