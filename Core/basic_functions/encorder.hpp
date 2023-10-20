#ifndef _ENCORDER_H_
#define _ENCORDER_H_
#include "machine_paramater.h"
class Encorders{
private:
	const float PPR=16384;//4096*4;
	const float gear_ratio=11.0/45.0;
	const float radius_mm;
	float pluse2mm;

	float period_ms;

	int pulseR;
	int pulseL;
	const int dirR=-1;
	const int dirL=1;

	void InitEncorderL();
	void InitEncorderR();


	float GetRPSL();
	float GetRPSR();

public:
	Encorders(int _period_ms);
	void InitEncorders();
	void Update();
	float GetVelociryL_mm_s();
	float GetVelociryR_mm_s();

	int GetPulseL();
	int GetPulseR();

};

#endif //_ENCORDER_H_