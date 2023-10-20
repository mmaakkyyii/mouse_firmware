#include "init_mpu.hpp"
#include "debug.hpp"
void init_sys(){
}

void init_delay_clk(){
}

void init_clk(){
}


void timer_start(){
	init_clk();
}

void init_mpu(){
	init_sys();
	
	init_delay_clk();

}
