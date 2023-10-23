#include "buzzer.hpp"
#include "static_parameters.h"
#include "main.h"
#include "tim.h"

Buzzer::Buzzer(int period_ms){
	update_period_ms=period_ms;
}
void Buzzer::Init(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	SetFrequency(400);
	Off();
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
	uint32_t period=0;
	__HAL_TIM_SET_AUTORELOAD(&htim2,period);
}
void Buzzer::On_ms(int f, int _time_ms){
	SetFrequency(f);
}

void Buzzer::On(){
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 250);

}
void Buzzer::Off(){
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}
