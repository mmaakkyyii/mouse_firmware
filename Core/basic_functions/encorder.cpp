#include "encorder.hpp"

Encorders::Encorders(int _period_ms):period_ms(_period_ms),pulseR(0),pulseL(0),radius_mm(TIYA_R){
	
}

void Encorders::InitEncorders(){
	InitEncorderL();
	InitEncorderR();
	pluse2mm =  1/(PPR)*3.14*gear_ratio*radius_mm;
	pulseR=0;
	pulseL=0;
	
}
void Encorders::InitEncorderL(){
}
void Encorders::InitEncorderR(){
}
void Encorders::Update(){
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
	return (float)pulseL/PPR*3.14*gear_ratio*radius_mm/(period_ms/1000.0);//�v�Z��������
}
float Encorders::GetVelociryR_mm_s(){
	return (float)pulseR/PPR*3.14*gear_ratio*radius_mm/(period_ms/1000.0);
}
