#include "IMU.hpp"
#include "static_parameters.h"
#include "delay.h"

//MISO P17
//MOSI PC6
//CLK  PC5
//CS   PC4

IMU::IMU(){
	calibration_flag=false;
	calibration_finish_flag=false;
}

void IMU::Init(){
	
}
bool IMU::Calibration(){
	if(calibration_flag==false){
		calibration_flag=true;
		calibration_finish_flag=false;
		gyro_data_offset[2]=0;
		cal_num=0;
	}
	const int offset_num=512;

//	unsigned long _imu=Read16bit(GYRO_XOUT_H_ADDR+2*2);
//	gyro_data_offset[2]+=(_imu>>8) & 0xffff;
	if(cal_num>offset_num){
		gyro_data_offset[2]=gyro_data_offset[2]/cal_num;
		calibration_finish_flag=true;
		calibration_flag=false;
	}
	return calibration_finish_flag;
}

unsigned short IMU::Read16bit(unsigned char  addr){
	return 0;
}

unsigned char IMU::Read(unsigned char  addr){

}


unsigned long IMU::Write(unsigned char  addr, char data){
	return 0;
}

void IMU::Update(){
	unsigned long _imu;
	_imu=Read16bit(GYRO_XOUT_H_ADDR+2*2);

	if(calibration_flag){
		gyro_data[2]=-1;
		gyro_data_offset[2]+=(signed short)(_imu);
		cal_num++;
	}else{
		gyro_data[2]=(signed short)(_imu);
		gyro_data[2]-=gyro_data_offset[2];
	}
	
}
void IMU::GetGyroRaw(int * gyro){
	gyro[0]=gyro_data[0];
	gyro[1]=gyro_data[1];
	gyro[2]=gyro_data[2];
}
void IMU::GetAccRaw(short * acc){
	acc[0]=acc_data[0];
	acc[1]=acc_data[1];
	acc[2]=acc_data[2];
}
void IMU::GetGyro(float * gyro){
	gyro[0]=(float)gyro_data[0]/32768.0*GYRO_FS;
	gyro[1]=(float)gyro_data[1]/32768.0*GYRO_FS;
	gyro[2]=(float)gyro_data[2]/32768.0*GYRO_FS;
}
void IMU::GetAcc(float * acc){
	acc[0]=(float)acc_data[0]/acc_sensitivity;
	acc[1]=(float)acc_data[1]/acc_sensitivity;
	acc[2]=(float)acc_data[2]/acc_sensitivity;
}

