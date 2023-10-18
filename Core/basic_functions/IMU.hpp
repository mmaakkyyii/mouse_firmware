#ifndef _IMU_H_
#define _IMU_H_

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

#endif //_IMU_H_