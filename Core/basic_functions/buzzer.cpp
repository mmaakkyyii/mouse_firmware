#include "buzzer.hpp"
#include "main.h"
#include "tim.h"

Buzzer::Buzzer(int period_ms):update_period_ms(period_ms),period(100){
}
void Buzzer::Init(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	SetFrequency(400);
	Off();
}

int Buzzer::Update(){
	if(time_ms<set_time_ms){
		time_ms+=update_period_ms;
		On();
		return 1;
	}else{
		Off();
		return 0;
	}
	return -1;
}
void Buzzer::SetFrequency(int f){
//	period=1e6/f;//170e6/170/f

    period = (HAL_RCC_GetPCLK1Freq() / (htim2.Init.Prescaler + 1)) / (f - 1);
    htim2.Instance->CNT = 0 ;
	htim2.Instance->ARR = period; // 新しい周期を設定
    htim2.Instance->CCR3 = period / 3; // Duty cycleを50%に保持
}
void Buzzer::On_ms(int f, int _time_ms){
	SetFrequency(f);
	On();
	time_ms=0;
	set_time_ms=_time_ms;
}

void Buzzer::On(){
    htim2.Instance->CCR3 = period / 3; // Duty cycleを50%に保持

}
void Buzzer::Off(){
    htim2.Instance->CCR3 = 0; // Duty cycleを0%に保持
}
