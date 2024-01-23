#include "flash_util.hpp"

#include "main.h"
#include <cstring>
#include "debug.hpp"

const uint32_t start_addr=0x0807F800;
uint8_t flash_data[MAZESIZE_X*MAZESIZE_Y];

void FlashInit(){

}
uint32_t FlashEraseData(){
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase=FLASH_TYPEERASE_PAGES;
	erase.Banks=FLASH_BANK_2;
	erase.Page=127;
	erase.NbPages=1;

	uint32_t error=0;
	HAL_FLASHEx_Erase(&erase, &error);

	HAL_FLASH_Lock();
	return error;

}
void FlashGetMazeData(int data[MAZESIZE_X][MAZESIZE_Y]){

	memcpy(flash_data,(uint32_t*)start_addr,MAZESIZE_X*MAZESIZE_Y);
	for(int y=0;y<MAZESIZE_Y;y++){
		for(int x=0;x<MAZESIZE_X;x++){
			data[x][y]=flash_data[x+y*MAZESIZE_X];
		}
	}
}
void FlashSetMazeData(int data[MAZESIZE_X][MAZESIZE_Y]){
	FlashEraseData();
	HAL_FLASH_Unlock();
	int i=0;
	for(int y=0;y<MAZESIZE_Y;y++){
		uint64_t data64bit1=0;
		uint64_t data64bit2=0;
		for(int x=0;x<MAZESIZE_X;x++){
			if(x<8)data64bit1|=(uint64_t)data[x][y]<<(8*(x));
			else data64bit2|=(uint64_t)data[x][y]<<(8*(x-8));
//			i++;
		}
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr+1*(y*MAZESIZE_X), data64bit1);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr+1*(y*MAZESIZE_X)+8, data64bit2);
	}
	HAL_FLASH_Lock();
}
void FlashPrintMazeData(int data[MAZESIZE_X][MAZESIZE_Y]){
	for(int y=MAZESIZE_Y-1;y>=0;y--){
		for(int x=0;x<MAZESIZE_X;x++){
			printf("%3d ",data[x][y]);
			HAL_Delay(1);
		}
		printf("\r\n");
		HAL_Delay(1);
	}

}

