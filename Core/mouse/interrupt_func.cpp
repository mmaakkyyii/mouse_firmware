#include "interrupt_func.h"
#include "mouse.hpp"
#include "MachineMode.hpp"

extern Mouse ita_PiCo;
extern MachineMode* mode; 
void int_1ms(){
	ita_PiCo.Interrupt_1ms();
	mode->CheckBattery();	
	mode->Interrupt_1ms();

	if(mode->GetCurrentMode()!=mode->GetNextMode()){
		switch(mode->GetNextMode()){
		case idle_mode:
			delete mode;
			mode = new Idle(&ita_PiCo);
			mode->Init();
			break;
		case lowBattery_mode:
			delete mode;
			mode = new LowBattery(&ita_PiCo);
			mode->Init();
			break;
		case modeSelect_mode:
			delete mode;
			mode = new ModeSelect(&ita_PiCo);
			mode->Init();
			break;
		case serchRun_mode:
			delete mode;
			mode = new SerchRun(&ita_PiCo);
			mode->Init();
			break;
		case fastRun_mode:
			delete mode;
			mode = new FastRun(&ita_PiCo);
			mode->Init();
			break;
		case parameterSetting_mode:
			delete mode;
			mode = new ParameterSetting(&ita_PiCo);
			mode->Init();
			break;
		case sensorCheck_mode:
			delete mode;
			mode = new SensorCheck(&ita_PiCo);
			mode->Init();
			break;
		case debug_mode:
			delete mode;
			mode = new Debug(&ita_PiCo);
			mode->Init();
			break;
		default:
			break;
		}
		
	}

}

void int_125ums(){
	ita_PiCo.Interrupt_125us();
}

void int_10ms(){
	ita_PiCo.Interrupt_10ms();
}
