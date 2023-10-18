#include "localization.hpp"
#include "mathf.h"
#include "machine_paramater.h"

Localization::Localization(float _x, float _y, float _theta, float _period, Encorders* _encorders)
:x(_x),y(_y),theta(_theta),period(_period),encorders(_encorders){
	
}
void Localization::Update(){

	float dr=encorders->GetPulseR()*0.00004684787326388889*TIYA_R;
	float dl=encorders->GetPulseL()*0.00004684787326388889*TIYA_R;
	theta+=(dr-dl)/TREAD_WIDTH;
	x-=((dr+dl)/2.0)*sinf(theta);
	y+=((dr+dl)/2.0)*cosf(theta);
	
}
void Localization::GetPosition(float* _x, float* _y, float* _theta){
	*_x=x;
	*_y=y;
	*_theta=theta;
}
float Localization::GetTheta(){
	return theta;
}