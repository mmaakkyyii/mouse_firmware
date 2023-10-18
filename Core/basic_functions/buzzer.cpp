#include "buzzer.hpp"
#include "iodefine.h"
#include "static_parameters.h"

Buzzer::Buzzer(int period_ms){
	update_period_ms=period_ms;
}
void Buzzer::Init(){
	PORT1.PDR.BIT.B6 = 1;
	PORT1.PODR.BIT.B6 = 0;

	SYSTEM.PRCR.WORD = 0xA502;//プロテクト解除
	MSTP(MTU) = 0;            //MTUモジュールON
	SYSTEM.PRCR.WORD = 0xA500;//プロテクト
	MTU.TSTR.BYTE=0;          //タイマ動作ストップ
	
	MTU3.TCR.BIT.CCLR=6;	    //PWM TGRDのコンペアマッチでTCNTクリア 
	MTU3.TCR.BIT.TPSC=1;	    //PCLK/4 12MHz
	MTU3.TMDR.BIT.MD=2;	    //PWM1
	MTU3.TIORL.BIT.IOC=1;	    //コンベアマッチてlow初期はlow
	MTU3.TIORL.BIT.IOD=2;	    //コンベアマッチでhigh
	MTU3.TGRC = 6000;         //1kHz
	MTU3.TGRD = (12000-1);
	  
	MTU.TSTR.BIT.CST3 = 0;    //タイマストップ
	  
	MPC.PWPR.BIT.B0WI=0;      //プロテクト解除
	MPC.PWPR.BIT.PFSWE=1;
	MPC.P16PFS.BIT.PSEL=1;    //機能の選択　ここではMTU
	MPC.PWPR.BYTE=0x80;       //プロテクト
	  
	PORT1.PMR.BIT.B6=1;	    //GPIO -> Buzzer PWM  

	MTU.TSTR.BIT.CST3 = 0;	//タイマストップ
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
	MTU3.TGRD=(unsigned short)(12000000/(f));
	MTU3.TGRC=(unsigned short)(3000000/(f));	//ブザーの発振周波数を算出して、設定
}
void Buzzer::On_ms(int f, int _time_ms){
	SetFrequency(f);
	set_time_ms = _time_ms;
	time_ms=0;
	MTU3.TCNT =0;

	PORT1.PMR.BIT.B6=1;
	MTU.TSTR.BIT.CST3=1;
}

void Buzzer::On(){
	MTU3.TCNT =0;

	PORT1.PMR.BIT.B6=1;
	MTU.TSTR.BIT.CST3=1;
}
void Buzzer::Off(){
	PORT1.PMR.BIT.B6=0;
	MTU.TSTR.BIT.CST3=0;
}
