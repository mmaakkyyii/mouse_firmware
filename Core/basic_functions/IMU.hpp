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
class IMU{
private:
	unsigned long Write(unsigned char  addr, char data);
	unsigned char Read(unsigned char  addr);
	unsigned short Read16bit(unsigned char  addr);
	const int gyro_sensitivity = 131;//131 LSB/(deg/s)
	const int acc_sensitivity = 2;//2/ LSB/(m/s)
	
	const int GYRO_CONFIG_ADDR = 0x1b;
	const int ACCEL_XOUT_H_ADDR = 0x3b;
	const int GYRO_XOUT_H_ADDR = 0x43;

	int gyro_data[3];
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