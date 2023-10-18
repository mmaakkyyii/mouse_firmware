#ifndef _MOTOR_H_
#define _MOTOR_H_
class Motors{
private:
	float Vin;

public:
	Motors();
	void Init();
	void InitMotorL();
	void InitMotorR();

	void SetDutyPWMR(unsigned short duty);
	void SetDutyPWML(unsigned short duty);
	void SetVoltageR(float v);
	void SetVoltageL(float v);

};

#endif //_MOTOT_H_