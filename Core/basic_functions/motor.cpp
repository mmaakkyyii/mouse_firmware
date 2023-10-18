#include "motor.hpp"
#include "iodefine.h"
#include "static_parameters.h"

Motors::Motors(){
	Vin=8.0;
}
void Motors::Init(){
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(MTU) = 0;			//MTUモジュールON
    	SYSTEM.PRCR.WORD = 0xA500;	
	
	MTU.TSTR.BYTE=0;		//タイマ動作ストップ

	InitMotorL();
	InitMotorR();
	MTU.TSTR.BIT.CST0 = 1;	//タイマスタート
	SetDutyPWMR(0);
	SetDutyPWML(0);

}
void Motors::InitMotorR(){
	
	PORTB.PDR.BIT.B3 = IO_OUT;//PWM_L
	PORTA.PDR.BIT.B0 = IO_OUT;// PA0:DIR_L

	SYSTEM.PRCR.WORD = 0xA502;//プロテクト解除
	MSTP(MTU) = 0;            //MTUモジュールON
	SYSTEM.PRCR.WORD = 0xA500;//プロテクト
	MTU.TSTR.BYTE=0;          //タイマ動作ストップ

	MTU0.TCR.BIT.CCLR=2;	    //PWM TGRBのコンペアマッチでTCNTクリア 
	MTU0.TCR.BIT.TPSC=0;	    //PCLK/1 **48MHz**
	MTU0.TMDR.BIT.MD=2;	    //PWM1
	MTU0.TIORH.BIT.IOA=1;	    //コンベアマッチてlow初期はlow
	MTU0.TIORH.BIT.IOB=2;	    //コンベアマッチでhigh
	MTU0.TGRA = 0;         //10kHz
	MTU0.TGRB = (1200-1);//12M/12000=10000Hz
	MTU0.TCNT = 0;

	MPC.PWPR.BIT.B0WI=0;
	MPC.PWPR.BIT.PFSWE=1;

	//MPC.PA0PFS.BIT.PSEL=1;	//MTIOC4A PA0:DIR_L
	MPC.PB3PFS.BIT.PSEL=1;	//MTIOC0A PB3:PWM_L

	PORTA.PMR.BIT.B0=0;		//DIR_L
	PORTB.PMR.BIT.B3=1;		//PWM_L

	

}
void Motors::InitMotorL(){
		
	PORTB.PDR.BIT.B0=IO_OUT;			//DIR_R
	PORTB.PDR.BIT.B1=IO_OUT;			//PWM_R

	SYSTEM.PRCR.WORD = 0xA502;//プロテクト解除
	MSTP(MTU) = 0;            //MTUモジュールON
	SYSTEM.PRCR.WORD = 0xA500;//プロテクト
	MTU.TSTR.BYTE=0;          //タイマ動作ストップ

	MTU0.TCR.BIT.CCLR=6;	    //PWM TGRBのコンペアマッチでTCNTクリア 
	MTU0.TCR.BIT.TPSC=0;	    //PCLK/1 **48MHz**
	MTU0.TMDR.BIT.MD=2;	    //PWM1
	MTU0.TIORL.BIT.IOC=1;	    //コンベアマッチてlow初期はlow
	MTU0.TIORL.BIT.IOD=2;	    //コンベアマッチでhigh
	MTU0.TGRC = 0;         //10kHz
	MTU0.TGRD = (1200-1);//12M/12000=10000Hz
	MTU0.TCNT = 0;

	MPC.PWPR.BIT.B0WI=0;
	MPC.PWPR.BIT.PFSWE=1;

	//MPC.PB0PFS.BIT.PSEL=3;	//TIOCA3  PB1:DIR_R
	MPC.PB1PFS.BIT.PSEL=1;	//MTIOC0C  PB1:PWM_R


	PORTB.PMR.BIT.B0=0;		//DIR_R
	PORTB.PMR.BIT.B1=1;		//PWM_R

}


void Motors::SetDutyPWMR(unsigned short duty){//max 1200
	if(duty<=0){
	//MTU.TSTR.BIT.CST0=0;
	MTU0.TGRA=0;
	}else{
	//MTU0.TCNT = 0;
	MTU0.TGRA=duty;
	//	MTU.TSTR.BIT.CST0=1;
	}
}
void Motors::SetDutyPWML(unsigned short duty){
	if(duty<=0){
	//MTU.TSTR.BIT.CST0=0;
	MTU0.TGRC=0;
	}else{
	//MTU0.TCNT = 0;
	MTU0.TGRC=duty;
	//	MTU.TSTR.BIT.CST0=1;
	}
}


void Motors::SetVoltageR(float v){
	int duty=(int)((v/Vin)*1200);
	if(duty>0){
		//SetDutyDirR(0);
		SetDutyPWMR(duty);
		PORTA.PODR.BIT.B0=0;
	}else if(duty<0){
		//SetDutyDirR(-duty);
		SetDutyPWMR(-duty);
		PORTA.PODR.BIT.B0=1;
	}else{
		SetDutyPWMR(0);
	}
}
void Motors::SetVoltageL(float v){
	int duty=(int)((v/Vin)*1200);
	if(duty>0){
		//SetDutyDirL(duty);
		SetDutyPWML(duty);
		//PORTB.PODR.BIT.B3=0;
		PORTB.PODR.BIT.B0=1;
	}else if(duty<0){
		//SetDutyDirL(-duty);
		SetDutyPWML(-duty);
		//PORTB.PODR.BIT.B3=1;
		PORTB.PODR.BIT.B0=0;
	}else{
		SetDutyPWML(0);
	}
}
