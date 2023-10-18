#include "wall_sensor.hpp"
//#include "portdef.h"
#include "iodefine.h"
#include "static_parameters.h"

#define SLED_L	(PORTB.PODR.BIT.B5)			//左センサLED
#define SLED_R	(PORT2.PODR.BIT.B7)			//右センサLED
#define SLED_FL	(PORT0.PODR.BIT.B5)			//左前センサLED
#define SLED_FR	(PORT5.PODR.BIT.B4)			//右前センサLED


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
	
	S12AD.ADCER.BIT.ADRFMT=0;//‰E?A?s
	S12AD.ADCSR.BIT.CKS=0x03;//PCLK?I?a?u?E?μ
//	S12AD.ADSSTR01.BIT.SST1=20;//Default 20?X?e?[?g 0.417us 0.4us?E?a?a???§

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
	static int state = 0;		//読み込むセンサのローテーション管理用変数
	int i;
	switch(state)
	{
		case 0:										//右センサ読み込み

			SLED_R = 1;								//LED点灯
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//フォトトランジスタの応答待ちループ
			S12AD.ADANS0.BIT.ANS0=0x0040;			//AN006
			S12AD.ADCSR.BIT.ADST=1;					//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			SLED_R = 0;								//LED消灯

			pre_right = right;			//過去の値を保存
			right = S12AD.ADDR6;				//値を保存

			if(right > TH_SEN_R)			//壁の有無を判断
			{
				is_wallR = 1;				//右壁あり
			}
			else
			{
				is_wallR = 0;				//右壁なし
			}
			
			if(right > TH_CTRL_R)		//制御をかけるか否かを判断
			{
				error_R = right - REF_SEN_R;	//制御をかける場合は偏差を計算
				is_controlR = 1;			//右センサを制御に使う
			}
			else
			{
				error_R = 0;					//制御に使わない場合は偏差を0にしておく
				is_controlR = 0;			//右センサを制御に使わない
			}			
			break;


		case 1:		//前左センサ読み込み

			SLED_FL = 1;							//LED点灯
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//フォトトランジスタの応答待ちループ
			S12AD.ADANS0.BIT.ANS0=0x0010;			//AN004
			S12AD.ADCSR.BIT.ADST=1;					//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			SLED_FL = 0;							//LED消灯

			pre_frontL = frontL;			//過去の値を保存
			frontL = S12AD.ADDR4;				//値を保存

			if(frontL > TH_SEN_FL)		//壁の有無を判断
			{
				is_wallFL = 1;				//左前壁あり
			}
			else
			{
				is_wallFL = 0;				//左前壁なし
			}
			break;


		case 2:		//前右センサ読み込み
		
			SLED_FR = 1;							//LED点灯
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//フォトトランジスタの応答待ちループ
			S12AD.ADANS0.BIT.ANS0=0x0200;			//AN009
			S12AD.ADCSR.BIT.ADST=1;					//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			SLED_FR = 0;							//LED消灯

			pre_frontR = frontR;			//過去の値を保存
			frontR = S12AD.ADDR9;				//値を保存

			if(frontR > TH_SEN_FR)		//壁の有無を判断
			{
				is_wallFR = 1;				//右前壁あり
			}
			else
			{
				is_wallFR = 0;				//右前壁なし
			}			
			break;


		case 3:		//左センサ読み込み
		
			SLED_L = 1;					//LED点灯
			for(i = 0; i < WAITLOOP_SLED; i++)	;	//フォトトランジスタの応答待ちループ
			S12AD.ADANS0.BIT.ANS0=0x0004;			//AN002
			S12AD.ADCSR.BIT.ADST=1;					//AD変換開始
			while(S12AD.ADCSR.BIT.ADST);			//AD変換終了まで待つ
			SLED_L = 0;					//LED消灯

			pre_left = left;			//過去の値を保存
			left = S12AD.ADDR2;				//値を保存
			
			if(left > TH_SEN_L)			//壁の有無を判断
			{
				is_wallL = 1;				//左壁あり
			}
			else
			{
				is_wallL = 0;				//左壁なし
			}
			
			if(left > TH_CTRL_L)		//制御をかけるか否かを判断
			{
				error_L = left - REF_SEN_L;	//制御をかける場合は偏差を計算する
				is_controlL = 1;			//左センサを制御に使う
			}
			else
			{
				error_L = 0;					//制御に使わない場合は偏差を0にしておく
				is_controlL = 0;			//左センサを制御に使わない
			}

			break;
	}
	
	state++;		//４回ごとに繰り返す
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