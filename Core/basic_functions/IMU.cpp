#include "IMU.hpp"
//#include "portdef.h"
#include "iodefine.h"
#include "static_parameters.h"
//MISO P17
//MOSI PC6
//CLK  PC5
//CS   PC4

int Gyro_Sensitivity = 131;//131 LSB/(deg/s)

int gyro_data[3];
int acc_data[3];
void InitIMU(){
	SYSTEM.PRCR.WORD = 0xA502;
 	MSTP(RSPI0) = 0;
	SYSTEM.PRCR.WORD = 0xA500;

	RSPI0.SPPCR.BIT.SPLP = 0; //RSPI???[?v?o?b?N?r?b?g ?fE?i???[?h
	RSPI0.SPPCR.BIT.SPLP2 = 0; //RSPI2???[?v?o?b?N?r?b?g ?fE?i???[?h
	RSPI0.SPPCR.BIT.MOIFV = 0; //MOSI?A?C?h???A?fe?fl?r?b?g ?A?C?h????Low
	RSPI0.SPPCR.BIT.MOIFE = 0; //MOSI?A?C?h???fl?A?fe???nA?r?b?g?@?eO?nn?g]?e??I?A?I?f?[?^
	
	RSPI0.SPBR = 3;
	RSPI0.SPDCR.BIT.SPFC=0;//number of frame
	RSPI0.SPDCR.BIT.SPRDTD=0;//read rx buffer
	RSPI0.SPDCR.BIT.SPLW=1;//ward access for SPDR


	RSPI0.SPCKD.BIT.SCKDL=2; //?N???b?N?fx?n? 1RSPCK
	RSPI0.SSLND.BIT.SLNDL=1;//SSL?l?Q?[?g?fx?n? 1PSPCK
	RSPI0.SPND.BIT.SPNDL=2;//RSPI???A?N?Z?X?fx?n? 1RSPCK+2PCLK
	
	RSPI0.SPCR2.BIT.SPPE=0;
	RSPI0.SPCR2.BIT.SPOE=0;
	RSPI0.SPCR2.BIT.SPIIE=0;
	RSPI0.SPCR2.BIT.PTE=0;
	
	RSPI0.SPCMD0.BIT.CPHA=1; //SPI mode 3
	RSPI0.SPCMD0.BIT.CPOL=1; //SPI mode 3
	
	RSPI0.SPCMD0.BIT.BRDV=3; //bit rate
	RSPI0.SPCMD0.BIT.SSLA=0; //SSL0
	RSPI0.SPCMD0.BIT.SPB=0x0f; //16bit
	RSPI0.SPCMD0.BIT.LSBF=0;//MSB first
	RSPI0.SPCMD0.BIT.SPNDEN=0;
	RSPI0.SPCMD0.BIT.SLNDEN=0;
	RSPI0.SPCMD0.BIT.SCKDEN=0;
	
	//MISO P17
	//MOSI PC6
	//CLK  PC5
	//CS   PC4
	PORT1.PDR.BIT.B7=IO_IN;
	PORTC.PDR.BIT.B6=IO_OUT;
	PORTC.PDR.BIT.B5=IO_OUT;
	PORTC.PDR.BIT.B4=IO_OUT;

	MPC.PWPR.BIT.B0WI=0;
	MPC.PWPR.BIT.PFSWE=1;

	MPC.P17PFS.BIT.PSEL=13;//MISOA
	MPC.PC6PFS.BIT.PSEL=13;//MOSIA
	MPC.PC5PFS.BIT.PSEL=13;	//RSPCKA 
	MPC.PC4PFS.BIT.PSEL=13;	//SSLA0  
	MPC.PWPR.BYTE = 0x80;  // Reprotect

	PORT1.PMR.BIT.B7=1;
	PORTC.PMR.BIT.B6=1;
	PORTC.PMR.BIT.B5=1;
	PORTC.PMR.BIT.B4=1;

	RSPI0.SPCR.BIT.SPMS=0;//SPI operation
	RSPI0.SPCR.BIT.TXMD=0;//full duplex
	RSPI0.SPCR.BIT.MODFEN=0;//disable mode fault
	RSPI0.SPCR.BIT.MSTR=1;//master mode
	RSPI0.SPCR.BIT.SPEIE=0;//disable interrupt request
	RSPI0.SPCR.BIT.SPTIE=0;//disable interrupt
	RSPI0.SPCR.BIT.SPE=1;//enable RSPI
	RSPI0.SPCR.BIT.SPRIE=0;//disable spi recive interrupt
	
}

unsigned short Read16bitIMU(unsigned char  addr){
	RSPI0.SPCMD0.BIT.SPB=0x03; //32bit
	RSPI0.SPCR.BIT.SPE=1;
	RSPI0.SPDR.LONG=((addr<<24)|0x80<<24) & 0xFF000000;
//	RSPI0.SPCR.BIT.SPTIE  = 0;  // Disable transmission IRQ
//	RSPI0.SPCR2.BIT.SPIIE = 1;  // Enable idle IRQ
//	RSPI0.SPCR.BIT.SPRIE  = 1;  // Enable receive IRQ
	
	RSPI0.SPDCR.BIT.SPRDTD=0;//rx buffer
	long data =RSPI0.SPDR.LONG;
	return (data)&0xFFFF;
}

unsigned char ReadIMU(unsigned char  addr){
	RSPI0.SPCMD0.BIT.SPB=0x03; //32bit
	RSPI0.SPCR.BIT.SPE=1;
	RSPI0.SPDR.LONG=((addr<<24)|0x80<<24)& 0xFF000000;
//	RSPI0.SPCR.BIT.SPTIE  = 0;  // Disable transmission IRQ
//	RSPI0.SPCR2.BIT.SPIIE = 1;  // Enable idle IRQ
//	RSPI0.SPCR.BIT.SPRIE  = 1;  // Enable receive IRQ
	
	RSPI0.SPDCR.BIT.SPRDTD=0;//rx buffer
	long data =RSPI0.SPDR.LONG;
	return (data>>16)&0xFF;

}


unsigned long WriteIMU(){
	RSPI0.SPCMD0.BIT.SPB=0x0f; //16bit

	RSPI0.SPCR.BIT.SPE=1;
	RSPI0.SPDR.LONG=0xf5000000 & 0xFF000000;
	
//	RSPI0.SPCR.BIT.SPTIE  = 0;  // Disable transmission IRQ
//	RSPI0.SPCR2.BIT.SPIIE = 1;  // Enable idle IRQ
//	RSPI0.SPCR.BIT.SPRIE  = 1;  // Enable receive IRQ
	
	RSPI0.SPDCR.BIT.SPRDTD=0;//rx buffer
	return (RSPI0.SPDR.LONG);

}

void UpdateIMU(){
}
void GetGyroRaw(int * gyro){
}
void GetAccRaw(int * acc){
}
void GetGyro(float * gyro){}
void GetAcc(float * acc){}

