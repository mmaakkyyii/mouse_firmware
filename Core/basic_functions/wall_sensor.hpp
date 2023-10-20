#ifndef _WALL_SENSOR_HPP
#define _WALL_SENSOR_HPP

class WallSensor{
public:
	WallSensor();
	void Init();
	void Update();
	int GetRight();
	int GetLeft();
	int GetFrontR();
	int GetFrontL();
	int GetErrorR(){return error_R;}
	int GetErrorL(){return error_L;}
	int GetError();
	int GetWallR(){return is_wallR;}
	int GetWallL(){return is_wallL;}
	int GetWallFR(){return is_wallFR;}
	int GetWallFL(){return is_wallFL;}

private:
	static const int WAITLOOP_SLED = 300;	//LED�����点�Ă���AD�ϊ����J�n����܂ł̎��ԉ҂��p�萔

	static const int REF_SEN_R = 451;//380;	//�}�E�X����H�����ɒu�������̃Z���T�̒l ���(451)
	static const int REF_SEN_L = 435;//413;	//�}�E�X����H�����ɒu�������̃Z���T�̒l ���(435)

	static const int TH_CTRL_R = 200;	//�ǂ����邩�ۂ���臒l
	static const int TH_CTRL_L = 200;	//�ǂ����邩�ۂ���臒l

	static const int TH_SEN_R = 150;	//�ǂ����邩�ۂ���臒l
	static const int TH_SEN_L = 155;	//�ǂ����邩�ۂ���臒l
	static const int TH_SEN_FR = 200;	//�ǂ����邩�ۂ���臒l
	static const int TH_SEN_FL = 170;	//�ǂ����邩�ۂ���臒l



	int right;
	int left;
	int frontR;
	int frontL;
	int pre_right;
	int pre_left;
	int pre_frontR;
	int pre_frontL;
	int error_R;
	int error_L;
	bool is_wallR;
	bool is_wallL;
	bool is_wallFR;
	bool is_wallFL;
	bool is_controlR;
	bool is_controlL;
};
#endif //_WALL_SENSOR_HPP