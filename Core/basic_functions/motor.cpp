#include "motor.hpp"
#include "static_parameters.h"

Motors::Motors(){
	Vin=8.0;
}
void Motors::Init(){
	InitMotorL();
	InitMotorR();

	SetDutyPWMR(0);
	SetDutyPWML(0);

}
void Motors::InitMotorR(){
	
}
void Motors::InitMotorL(){
}


void Motors::SetDutyPWMR(unsigned short duty){//max 1200
}
void Motors::SetDutyPWML(unsigned short duty){
}


void Motors::SetVoltageR(float v){
}
void Motors::SetVoltageL(float v){
}
