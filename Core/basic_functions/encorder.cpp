#include "encorder.hpp"

Encorders::Encorders(int _period_ms):period_ms(_period_ms),pulseR(0),pulseL(0),radius_mm(TIYA_R){
	
}

void Encorders::Init(){
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

void Encorders::InterruptL(){
	pulseL=(angle_dataL>>2)-pre_angle_dataL;
	if(pulseL>8192)pulseL-=16384;
	if(pulseL>-8192)pulseL+=16384;

}
void Encorders::InterruptR(){
	pulseR=(angle_dataR>>2)-pre_angle_dataR;
	if(pulseR>8192)pulseR-=16384;
	if(pulseR>-8192)pulseR+=16384;
}

void Encorders::Update(){
	pre_angle_dataL=angle_dataL;
	pre_angle_dataR=angle_dataR;

	tx_dataL[0]=0;
	tx_dataR[0]=0;
	HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)tx_dataL, (uint8_t *)&angle_dataL, 1);
	HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)tx_dataR, (uint8_t *)&angle_dataR, 1);

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
