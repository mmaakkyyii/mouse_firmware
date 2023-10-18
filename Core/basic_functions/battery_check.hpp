#ifndef _BATTERY_CHECK_HPP_
#define _BATTERY_CHECK_HPP_

class BatteryCheck{
public:
	BatteryCheck();
	void Init();
	void Update();
	float GetBatteryVoltage_V();
private:
	int adc_val;
};

#endif //_BATTERY_CHECK_HPP_