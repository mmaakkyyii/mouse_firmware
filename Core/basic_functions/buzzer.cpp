#include "buzzer.hpp"
#include "static_parameters.h"

Buzzer::Buzzer(int period_ms){
	update_period_ms=period_ms;
}
void Buzzer::Init(){
}

int Buzzer::Update(){
	if(time_ms<set_time_ms){
		On();
		time_ms+=update_period_ms;
		return 1;
	}else{
		Off();
		return 0;
	}
	return -1;
}
void Buzzer::SetFrequency(int f){
}
void Buzzer::On_ms(int f, int _time_ms){
}

void Buzzer::On(){
}
void Buzzer::Off(){
}
