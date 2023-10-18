#include "wall_sensor.hpp"
//#include "portdef.h"
#include "iodefine.h"
#include "static_parameters.h"

#define SLED_L	(PORTB.PODR.BIT.B5)			//���Z���TLED
#define SLED_R	(PORT2.PODR.BIT.B7)			//�E�Z���TLED
#define SLED_FL	(PORT0.PODR.BIT.B5)			//���O�Z���TLED
#define SLED_FR	(PORT5.PODR.BIT.B4)			//�E�O�Z���TLED


WallSensor::WallSensor(){
	
}
void WallSensor::Init(){
	PORTB.PDR.BIT.B5 = IO_OUT;//SLED_L
	PORT2.PDR.BIT.B7 = IO_OUT;//SLED_R
	PORT0.PDR.BIT.B5 = IO_OUT;//SLED_FL
	PORT5.PDR.BIT.B4 = IO_OUT;//SLED_FR
	
	MPC.PE1PFS.BIT.ASEL=1;	//A/D SEN_FR
	MPC.P44PFS.BIT.ASEL=1;	//A/D SEN_FL
	MPC.P46PFS.BIT.ASEL=1;	//A/D SEN_R
	MPC.P42PFS.BIT.ASEL=1;	//A/D SEN_L
	MPC.PWPR.BYTE=0x80;
	
//	PORTE.PMR.BIT.B0=1;		//A/D
	PORTE.PMR.BIT.B1=1;		//A/D
	PORT4.PMR.BIT.B4=1;		//A/D
	PORT4.PMR.BIT.B6=1;		//A/D
	PORT4.PMR.BIT.B2=1;		//A/D

		SYSTEM.PRCR.WORD = 0xA502;
	MSTP(S12AD) = 0;
	    SYSTEM.PRCR.WORD = 0xA500;	
	
	S12AD.ADCER.BIT.ADRFMT=0;//��E?A?s
	S12AD.ADCSR.BIT.CKS=0x03;//PCLK?I?a?u?E?��
//	S12AD.ADSSTR01.BIT.SST1=20;//Default 20?X?e?[?g 0.417us 0.4us?E?a?a???��

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
	switch(state)
	{
		case 0:										//�E�Z���T�ǂݍ���

			SLED_R = 1;								//LED�_��
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//�t�H�g�g�����W�X�^�̉����҂����[�v
			S12AD.ADANS0.BIT.ANS0=0x0040;			//AN006
			S12AD.ADCSR.BIT.ADST=1;					//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			SLED_R = 0;								//LED����

			pre_right = right;			//�ߋ��̒l��ۑ�
			right = S12AD.ADDR6;				//�l��ۑ�

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


		case 1:		//�O���Z���T�ǂݍ���

			SLED_FL = 1;							//LED�_��
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//�t�H�g�g�����W�X�^�̉����҂����[�v
			S12AD.ADANS0.BIT.ANS0=0x0010;			//AN004
			S12AD.ADCSR.BIT.ADST=1;					//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			SLED_FL = 0;							//LED����

			pre_frontL = frontL;			//�ߋ��̒l��ۑ�
			frontL = S12AD.ADDR4;				//�l��ۑ�

			if(frontL > TH_SEN_FL)		//�ǂ̗L���𔻒f
			{
				is_wallFL = 1;				//���O�ǂ���
			}
			else
			{
				is_wallFL = 0;				//���O�ǂȂ�
			}
			break;


		case 2:		//�O�E�Z���T�ǂݍ���
		
			SLED_FR = 1;							//LED�_��
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//�t�H�g�g�����W�X�^�̉����҂����[�v
			S12AD.ADANS0.BIT.ANS0=0x0200;			//AN009
			S12AD.ADCSR.BIT.ADST=1;					//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			SLED_FR = 0;							//LED����

			pre_frontR = frontR;			//�ߋ��̒l��ۑ�
			frontR = S12AD.ADDR9;				//�l��ۑ�

			if(frontR > TH_SEN_FR)		//�ǂ̗L���𔻒f
			{
				is_wallFR = 1;				//�E�O�ǂ���
			}
			else
			{
				is_wallFR = 0;				//�E�O�ǂȂ�
			}			
			break;


		case 3:		//���Z���T�ǂݍ���
		
			SLED_L = 1;					//LED�_��
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//�t�H�g�g�����W�X�^�̉����҂����[�v
			S12AD.ADANS0.BIT.ANS0=0x0004;			//AN002
			S12AD.ADCSR.BIT.ADST=1;					//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST);			//AD�ϊ��I���܂ő҂�
			SLED_L = 0;					//LED����

			pre_left = left;			//�ߋ��̒l��ۑ�
			left = S12AD.ADDR2;				//�l��ۑ�
			
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