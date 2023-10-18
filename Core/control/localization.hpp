#ifndef _LOCALIZATION_HPP_
#define _LOCALIZATION_HPP_
#include "encorder.hpp"

class Localization{
public:
	Localization(float _x, float _y, float _theta, float _period, Encorders* _encorders);
	void Update();
	void GetPosition(float* _x, float* _y, float* _theta);
	float GetTheta();
private:
	Encorders* encorders;
	float period;
	float x;
	float y;
	float theta;

};
#endif //_LOCALIZATION_HPP_