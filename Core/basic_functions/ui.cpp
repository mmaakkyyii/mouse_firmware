#include "ui.hpp"
//#include "portdef.h"
#include "iodefine.h"
#include "static_parameters.h"


void UI::SetLED(int led_data)
{
	PORTC.PODR.BIT.B2 = led_data&0x01;
	PORT3.PODR.BIT.B1 = (led_data>>1)&0x01;
	PORT1.PODR.BIT.B5 = (led_data>>2)&0x01;
	PORTC.PODR.BIT.B3 = (led_data>>3)&0x01;
/*
#define LED0	(PORTC.PODR.BIT.B2)
#define LED2	(PORT1.PODR.BIT.B5)
#define LED3	(PORTC.PODR.BIT.B3)
#define LED1	(PORT3.PODR.BIT.B1)
*/
}

void UI::SetRBLED(int value){//0~100
	if(value<0)value=0;
	if(value>100)value=100;
	RBLED_value=value;
}

void UI::Init(){
	PORTC.PDR.BIT.B2 = 1;//LED0
	PORT1.PDR.BIT.B5 = 1;//LED1
	PORTC.PDR.BIT.B3 = 1;//LED2
	PORT3.PDR.BIT.B1 = 1;//LED3
	PORTB.PDR.BIT.B7 = 1;//BLED0
	PORTB.PDR.BIT.B6 = 1;//BLED1

}
void UI::Update(){
	static int count=0;
	count++;
	if(count>RBLED_value){
		PORTB.PODR.BIT.B7 = 0;//BLED0
		PORTB.PODR.BIT.B6 = 1;//BLED1
	}
	else{
		PORTB.PODR.BIT.B7 = 1;//BLED0
		PORTB.PODR.BIT.B6 = 0;//BLED1
	}
	if(count>100)count=0;
}
int UI::GetSW1(){
	return (PORTE.PIDR.BIT.B3);
}
int UI::GetSW2(){
	return (PORTE.PIDR.BIT.B5);
}
int UI::GetSW3(){
	return (PORTE.PIDR.BIT.B4);
}
