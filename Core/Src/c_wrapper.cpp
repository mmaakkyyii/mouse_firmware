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

PID_Controler PID_motorL(0.003,0.0005,0,1);
PID_Controler PID_motorR(0.003,0.0005,0,1);
Motors motors;
Encorders encorders(CONTROL_PERIOD_ms);
IMU imu;
Localization localization(0,0,0,1,&encorders);
WallSensor wall_sensor;
BatteryCheck battery_check;
Buzzer buzzer(1);
UI ui;
MazeSolver maze_solver;

Mouse mouse(&PID_motorR,&PID_motorL, &motors,&localization,&encorders,&imu,&wall_sensor,&battery_check,&buzzer,&ui,&maze_solver);
MachineMode* mode;



void Init(){

}

void Loop(){

}

void Interrupt1ms(){

}
void Interrupt100us(){

}
