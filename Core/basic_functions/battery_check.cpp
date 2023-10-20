#include "battery_check.hpp"
#include "adc.h"

BatteryCheck::BatteryCheck(){}
void BatteryCheck::Init(){
	HAL_ADC_Start_DMA(&hadc1, &adc_val, 1);

}
void BatteryCheck::Update(){
}
float BatteryCheck::GetBatteryVoltage_V(){
	return adc_val/4096.0*3.3 *(6.8+15.0)/(15.0);
}
