#include "wall_sensor.hpp"
#include "static_parameters.h"
#include "gpio.h"
#include "adc.h"

WallSensor::WallSensor(){
	
}
void WallSensor::Init(){
}

int WallSensor::GetError(){
	int error=0;
	if(is_controlR && is_controlL)error=GetErrorR()-GetErrorL();
	if(!is_controlR && is_controlL)error=2*(-GetErrorL());
	if(is_controlR && !is_controlL)error=2* (GetErrorR());
	if(!is_controlR && !is_controlL)error=0;
	return error;
}
void WallSensor::Update(){
	static int state = 0;		//�ǂݍ��ރZ���T�̃��[�e�[�V�����Ǘ��p�ϐ�
	int i;
	ADC_ChannelConfTypeDef sConfig = {0};
	switch(state)
	{
		case 0:
			HAL_GPIO_WritePin(LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_SET);
			sConfig.Channel = 4;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			HAL_ADC_Start(&hadc1); // ADC変換開始

			if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
				right= HAL_ADC_GetValue(&hadc1); // ADCの値を取得
			}
			HAL_GPIO_WritePin(LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_RESET);

//			SLED_R = 1;								//LED�_��
//			SLED_R = 0;								//LED����

			pre_right = right;			//�ߋ��̒l��ۑ�
//			right = S12AD.ADDR6;				//�l��ۑ�

			if(right > TH_SEN_R)			//�ǂ̗L���𔻒f
			{
				is_wallR = 1;				//�E�ǂ���
			}
			else
			{
				is_wallR = 0;				//�E�ǂȂ�
			}
			
			if(right > TH_CTRL_R)		//����������邩�ۂ��𔻒f
			{
				error_R = right - REF_SEN_R;	//�����������ꍇ�͕΍����v�Z
				is_controlR = 1;			//�E�Z���T�𐧌�Ɏg��
			}
			else
			{
				error_R = 0;					//����Ɏg��Ȃ��ꍇ�͕΍���0�ɂ��Ă���
				is_controlR = 0;			//�E�Z���T�𐧌�Ɏg��Ȃ�
			}			
			break;

		case 1:
//			SLED_FL = 1;							//LED�_��
//			SLED_FL = 0;							//LED����

			pre_frontL = frontL;			//�ߋ��̒l��ۑ�
//			frontL = S12AD.ADDR4;				//�l��ۑ�

			if(frontL > TH_SEN_FL)		//�ǂ̗L���𔻒f
			{
				is_wallFL = 1;				//���O�ǂ���
			}
			else
			{
				is_wallFL = 0;				//���O�ǂȂ�
			}
			break;

		case 2:
//			SLED_FR = 1;							//LED�_��
//			SLED_FR = 0;							//LED����

			pre_frontR = frontR;			//�ߋ��̒l��ۑ�
//			frontR = S12AD.ADDR9;				//�l��ۑ�

			if(frontR > TH_SEN_FR)		//�ǂ̗L���𔻒f
			{
				is_wallFR = 1;				//�E�O�ǂ���
			}
			else
			{
				is_wallFR = 0;				//�E�O�ǂȂ�
			}			
			break;

		case 3:
//			SLED_L = 1;					//LED�_��
//			SLED_L = 0;					//LED����

			pre_left = left;			//�ߋ��̒l��ۑ�
//			left = S12AD.ADDR2;				//�l��ۑ�
			
			if(left > TH_SEN_L)			//�ǂ̗L���𔻒f
			{
				is_wallL = 1;				//���ǂ���
			}
			else
			{
				is_wallL = 0;				//���ǂȂ�
			}
			
			if(left > TH_CTRL_L)		//����������邩�ۂ��𔻒f
			{
				error_L = left - REF_SEN_L;	//�����������ꍇ�͕΍����v�Z����
				is_controlL = 1;			//���Z���T�𐧌�Ɏg��
			}
			else
			{
				error_L = 0;					//����Ɏg��Ȃ��ꍇ�͕΍���0�ɂ��Ă���
				is_controlL = 0;			//���Z���T�𐧌�Ɏg��Ȃ�
			}

			break;
	}
	
	state++;		//�S�񂲂ƂɌJ��Ԃ�
	if(state > 3)
	{
		state = 0;
	}
		

}
int WallSensor::GetRight(){
	return right;
}
int WallSensor::GetLeft(){
	return left;
}
int WallSensor::GetFrontR(){
	return frontR;
}
int WallSensor::GetFrontL(){
	return frontL;
}
