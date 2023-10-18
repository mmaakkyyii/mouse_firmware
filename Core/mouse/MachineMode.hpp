#ifndef _MACHINE_MODE_HPP_
#define _MACHINE_MODE_HPP_

#include "mouse.hpp"

typedef enum {
	none_mode=-1,
	idle_mode=0,
	lowBattery_mode=1,
	modeSelect_mode=2,
	serchRun_mode=3,
	fastRun_mode=4,
	parameterSetting_mode=5,
	sensorCheck_mode=6,
	debug_mode=7
}ModeType;


class MachineMode{
public:
	MachineMode(Mouse* _mouse):mouse(_mouse){}
	 ~MachineMode(){};
	Mouse* mouse;
	virtual void Loop(){};
	virtual void Init(){};
	virtual void Interrupt_1ms(){};
	int IsOtherMode();
	void CheckBattery(){if(mouse->battery_check->GetBatteryVoltage_V() < 7)next_mode=lowBattery_mode;}
	ModeType GetCurrentMode(){return current_mode;}
	ModeType GetNextMode(){return next_mode;}
	
protected:
	ModeType current_mode;
	ModeType next_mode;
};

class Idle:public MachineMode{
public:
	void Loop(){};
	void Init(){};
	void Interrupt_1ms(){};
	Idle(Mouse* _mouse):MachineMode(_mouse){
	current_mode=idle_mode;
	next_mode=idle_mode;
	};
	~Idle();
};
class LowBattery:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	LowBattery(Mouse* _mouse);
	~LowBattery();
};
class ModeSelect:public MachineMode{
public:
	int mode_val;
	int sw1,sw2,sw3,pre_sw1,pre_sw2,pre_sw3;

	void Loop();
	void Init();
	void Interrupt_1ms();
	ModeSelect(Mouse* _mouse);
	~ModeSelect();
};
class SerchRun:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	SerchRun(Mouse* _mouse);
	~SerchRun(){
		delete trajectory;
	};
private:
	Trajectory* trajectory;

	float velocity_l,velocity_r;
	float target_velocity_l,target_velocity_r;

	float V_r;
	float V_l;
	
	float target_x,target_y,target_theta;
	float current_x,current_y,current_theta;
	float target_vx,target_vy,target_omega;
	float current_vx,current_vy,current_omega;

	int gesture_flag;
	int no_hand_flag;
	int timer;

	bool idle;
	
	
	//WallMask wall_mask;
};
class FastRun:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	FastRun(Mouse* _mouse);
	~FastRun(){
		delete trajectory;
	};
private:
	Trajectory* trajectory;

	float velocity_l,velocity_r;
	float target_velocity_l,target_velocity_r;

	float V_r;
	float V_l;
	
	float target_x,target_y,target_theta;
	float current_x,current_y,current_theta;
	float target_vx,target_vy,target_omega;
	float current_vx,current_vy,current_omega;

	int gesture_flag;
	int no_hand_flag;
	int timer;

	bool idle;
	
};
class ParameterSetting:public MachineMode{
public:
	void Loop(){};
	void Init(){};
	void Interrupt_1ms(){};
	ParameterSetting(Mouse* _mouse):MachineMode(_mouse){
	current_mode=parameterSetting_mode;
	next_mode=parameterSetting_mode;
	};
	~ParameterSetting();
};
class SensorCheck:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	SensorCheck(Mouse* _mouse);
	~SensorCheck();
};
class Debug:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	Debug(Mouse* _mouse);
	~Debug();
private:
	Trajectory* trajectory;

	float velocity_l,velocity_r;
	float target_velocity_l,target_velocity_r;

	float V_r;
	float V_l;
	
	float target_x,target_y,target_theta;
	float current_x,current_y,current_theta;
	float target_vx,target_vy,target_omega;
	float current_vx,current_vy,current_omega;

	int gesture_flag;
	int no_hand_flag;
	int timer;

	bool idle;
	
	
};

#endif //_MACHINE_MODE_HPP_