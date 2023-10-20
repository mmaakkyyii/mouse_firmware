#ifndef _BATTERY_CHECK_HPP_
#define _BATTERY_CHECK_HPP_

#include "stdint.h"

class BatteryCheck{
public:
	BatteryCheck();
	void Init();
	void Update();
	float GetBatteryVoltage_V();
private:
	uint32_t adc_val;
};

#endif //_BATTERY_CHECK_HPP_
