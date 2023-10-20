#include "battery_check.hpp"

BatteryCheck::BatteryCheck(){}
void BatteryCheck::Init(){

}
void BatteryCheck::Update(){
	adc_val = 0;				//�l��ۑ�

}
float BatteryCheck::GetBatteryVoltage_V(){
	return adc_val/4096.0*3.3 *(47.0+10.0)/(10.0);
}
