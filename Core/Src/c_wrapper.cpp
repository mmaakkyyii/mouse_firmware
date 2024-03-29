/*
 * c_wrapper.cpp
 *
 *  Created on: Oct 18, 2023
 *      Author: Makihara
 */

#include "c_wrapper.h"

#include "machine_paramater.h"

#include "PID.hpp"
#include "motor.hpp"
#include "encorder.hpp"
#include "IMU.hpp"
#include "localization.hpp"
#include "wall_sensor.hpp"
#include "battery_check.hpp"
#include "buzzer.hpp"
#include "ui.hpp"
#include "MazeSolver.hpp"
#include "MachineMode.hpp"
#include "interrupt_func.h"

PID_Controler PID_motorL(Kp_motorL, Ki_motorL, Kd_motorL, CONTROL_PERIOD_ms);
PID_Controler PID_motorR(Kp_motorR, Ki_motorR, Kd_motorR, CONTROL_PERIOD_ms);
Motors motors;
Encorders encorders(CONTROL_PERIOD_ms);
IMU imu;
Localization localization(0,0,0,CONTROL_PERIOD_ms,&encorders);
WallSensor wall_sensor;
BatteryCheck battery_check;
Buzzer buzzer(CONTROL_PERIOD_ms);
UI ui;
MazeSolver maze_solver;

Mouse mouse(&PID_motorR,&PID_motorL, &motors,&localization,&encorders,&imu,&wall_sensor,&battery_check,&buzzer,&ui,&maze_solver);
MachineMode* mode;



void Init(){
	mode=new ModeSelect(&mouse);
	mouse.Init();
}

void Loop(){
	//mouse.Loop();
	//mode->Loop();

}

void Interrupt125us(){
	int_125ums();
	//mouse.Interrupt_125us();
}
void Interrupt1ms(){
	int_1ms();
	//mouse.Interrupt_1ms();
}
void Interrupt10ms(){
	int_10ms();
	//mouse.Interrupt_10ms();
}
