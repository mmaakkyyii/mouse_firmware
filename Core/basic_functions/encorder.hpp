#ifndef _ENCORDER_H_
#define _ENCORDER_H_
#include "machine_paramater.h"
#include "spi.h"

class Encorders{
private:
	const float PPR=16384;//4096*4;
	const float gear_ratio=35.0/9.0;
	const float radius_mm;
	float pluse2mm;

	float period_ms;

	int pulseR;
	int pulseL;
	const int dirR=-1;
	const int dirL=1;

	uint16_t angle_dataL;
	uint16_t angle_dataR;
	uint16_t pre_angle_dataL;
	uint16_t pre_angle_dataR;

	uint16_t tx_dataL[2]={0,0};
	uint16_t tx_dataR[2]={0,0};


	void InitEncorderL();//SPI2
	void InitEncorderR();//SPI1


	float GetRPSL();
	float GetRPSR();
	void InterruptL();
	void InterruptR();

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
