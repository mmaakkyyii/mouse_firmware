#include "battery_check.hpp"
#include "iodefine.h"

BatteryCheck::BatteryCheck(){}
void BatteryCheck::Init(){
	MPC.PE0PFS.BIT.ASEL=1;	//A/D SEN_FR
	MPC.PWPR.BYTE=0x80;

	PORTE.PMR.BIT.B0=1;		//A/D

	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(S12AD) = 0;
	SYSTEM.PRCR.WORD = 0xA500;	

	S12AD.ADCER.BIT.ADRFMT=0;//‰E?A?s
	S12AD.ADCSR.BIT.CKS=0x03;//PCLK?I?a?u?E?μ

}
void BatteryCheck::Update(){
	S12AD.ADANS0.BIT.ANS0=0x0100;			//AN008
	S12AD.ADCSR.BIT.ADST=1;					//AD変換開始
	while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
	adc_val = S12AD.ADDR8;				//値を保存

}
float BatteryCheck::GetBatteryVoltage_V(){
	return adc_val/4096.0*3.3 *(47.0+10.0)/(10.0);
}