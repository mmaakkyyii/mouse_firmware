#ifndef _BUZZER_H_
#define _BUZZER_H_

#include "stdint.h"

class Buzzer{
public:
	Buzzer(int period_ms);
	void Init();
	int Update();
	void SetFrequency(int f);
	void On();
	void On_ms(int f,int _time_ms);
	void Off();
private:
	uint32_t period;
	int set_time_ms;
	int time_ms;
	int update_period_ms;
};


#endif //_BUZZER_H_


