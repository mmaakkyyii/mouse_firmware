#ifndef _IMU_H_
#define _IMU_H_
/*
void InitIMU();
unsigned long WriteIMU();
unsigned char ReadIMU(unsigned char  addr);
unsigned short Read16bitIMU(unsigned char  addr);

//Public///////////////
void UpdateIMU();
void GetGyroRaw(int * gyro);
void GetAccRaw(int * acc);
void GetGyro(float * gyro);
void GetAcc(float * acc);
*/

#include "stdint.h"
#include "spi.h"

class IMU{
private:
	void Write(uint8_t addr, uint8_t data);
	uint8_t Read(uint8_t addr);
	void Read(uint8_t addr, uint8_t* data);
	unsigned short Read16bit(unsigned char  addr);
	const int gyro_sensitivity = 131;//131 LSB/(deg/s)
	const int acc_sensitivity = 2;//2/ LSB/(m/s)
	
	const int GYRO_CONFIG_ADDR = 0x1b;
	const int ACCEL_XOUT_H_ADDR = 0x3b;
	const int TEMP_H_ADDR = 0x1D;
	const int GYRO_XOUT_H_ADDR = 0x25;
	const int WHO_AM_I_ADDR = 0x75;

	int16_t gyro_data[3];
	uint8_t gyro_data_8bit[14];
	int acc_data[3];
	const float GYRO_FS=2000;

	long int gyro_data_offset[3];
	long int acc_data_offset[3];
	
	bool calibration_flag;
	bool calibration_finish_flag;
	int cal_num;
	
public:
//MPU-9250
	IMU();
	void Init();
	void Update();
	bool Calibration();
	void GetGyroRaw(int * gyro);
	void GetAccRaw(short * acc);
	void GetGyro(float * gyro);
	void GetAcc(float * acc);
	short GetGzOffset(){return gyro_data_offset[2];}
	
};

#endif //_IMU_H_
