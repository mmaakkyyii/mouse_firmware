#include "encorder.hpp"
#include "iodefine.h"

//ENC_L A PA1
//ENC_L B PA3

//ENC_R A PA4
//ENC_R B PA6

Encorders::Encorders(int _period_ms):period_ms(_period_ms),pulseR(0),pulseL(0){
}

void Encorders::InitEncorders(){
	InitEncorderL();
	InitEncorderR();
	pluse2mm =  1/(PPR)*3.14*gear_ratio*radius_mm;
	pulseR=0;
	pulseL=0;
	
}
void Encorders::InitEncorderL(){
	PORTA.PMR.BIT.B1=1;	    //Uses the pin as an I/O port for peripheral functions.
	PORTA.PMR.BIT.B3=1;	    // Uses the pin as an I/O port for peripheral functions.

	
	SYSTEM.PRCR.WORD = 0xA502;//プロテクト解除
	MSTP(MTU) = 0;            //MTUモジュールON
	SYSTEM.PRCR.WORD = 0xA500;//プロテクト
	MTU.TSTR.BIT.CST1 = 0;	//タイマスタート

	MPC.PWPR.BIT.B0WI=0;      //プロテクト解除
	MPC.PWPR.BIT.PFSWE=1;
	
	MPC.PA1PFS.BIT.PSEL=2;	//MTCLKC
	MPC.PA3PFS.BIT.PSEL=2;	//MTCLKD
	
	MPC.PWPR.BYTE=0x80;       //プロテクト
	
	MTU2.TMDR.BIT.MD=4; //位相計数モード1
	MTU2.TCNT=0;
	MTU.TSTR.BIT.CST2 = 1;	//タイマスタート
}
void Encorders::InitEncorderR(){
	PORTA.PMR.BIT.B4=1;	    //Uses the pin as an I/O port for peripheral functions.
	PORTA.PMR.BIT.B6=1;	    // Uses the pin as an I/O port for peripheral functions.
	
	SYSTEM.PRCR.WORD = 0xA502;//プロテクト解除
	MSTP(MTU) = 0;            //MTUモジュールON
	SYSTEM.PRCR.WORD = 0xA500;//プロテクト
	MTU.TSTR.BIT.CST1 = 0;	//タイマスタート

	MPC.PWPR.BIT.B0WI=0;      //プロテクト解除
	MPC.PWPR.BIT.PFSWE=1;

	MPC.PA4PFS.BIT.PSEL=2;	//MTCLKA
	MPC.PA6PFS.BIT.PSEL=2;	//MTCLKB
	
	MPC.PWPR.BYTE=0x80;       //プロテクト
	
	MTU1.TMDR.BIT.MD=4; //位相計数モード1
	MTU1.TCNT=0;
	MTU.TSTR.BIT.CST1 = 1;	//タイマスタート
}
void Encorders::Update(){
	pulseR=dirR*(short)MTU1.TCNT;
	pulseL=dirL*(short)MTU2.TCNT;
	MTU1.TCNT=0;	
	MTU2.TCNT=0;
	
}

int Encorders::GetPulseL(){
	return pulseL;
}
int Encorders::GetPulseR(){
	return pulseR;
}

float Encorders::GetRPSL(){
	return pulseL/period_ms;
}
float Encorders::GetRPSR(){
	return pulseR/period_ms;
}


float Encorders::GetVelociryL_mm_s(){
	return (float)pulseL/PPR*3.14*gear_ratio*radius_mm/(period_ms/1000.0);//計算おかしい
}
float Encorders::GetVelociryR_mm_s(){
	return (float)pulseR/PPR*3.14*gear_ratio*radius_mm/(period_ms/1000.0);
}
