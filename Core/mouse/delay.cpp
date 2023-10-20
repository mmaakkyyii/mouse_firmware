#include "delay.h"

volatile int delay_counter;
void inc_delay_cont(){
	delay_counter++;
}
void delay_ms(int t_ms){
	delay_counter=0;
	while(delay_counter<t_ms){}

}	