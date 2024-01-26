#include "flash_util.hpp"

#include "main.h"
#include <cstring>
#include "debug.hpp"

const uint32_t start_addr=0x0807F800;
uint8_t flash_data[MAZESIZE_X*MAZESIZE_Y+param_data_num];

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
void FlashGetData(int maze_data[MAZESIZE_X][MAZESIZE_Y],int param_data[param_data_num]){

	memcpy(flash_data,(uint32_t*)start_addr,MAZESIZE_X*MAZESIZE_Y+param_data_num);
	for(int y=0;y<MAZESIZE_Y;y++){
		for(int x=0;x<MAZESIZE_X;x++){
			maze_data[x][y]=flash_data[x+y*MAZESIZE_X];
		}
	}
	for(int i=0;i<param_data_num;i++){
		param_data[i]=flash_data[MAZESIZE_X*MAZESIZE_Y+i];
	}
}
void FlashSetData(int maze_data[MAZESIZE_X][MAZESIZE_Y],int param_data[param_data_num]){
	FlashEraseData();
	HAL_FLASH_Unlock();
	for(int y=0;y<MAZESIZE_Y;y++){
		uint64_t data64bit1=0;
		uint64_t data64bit2=0;
		for(int x=0;x<MAZESIZE_X;x++){
			if(x<8)data64bit1|=(uint64_t)maze_data[x][y]<<(8*(x));
			else data64bit2|=(uint64_t)maze_data[x][y]<<(8*(x-8));
//			i++;
		}
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr+(y*MAZESIZE_X), data64bit1);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr+(y*MAZESIZE_X)+8, data64bit2);
	}
	uint64_t data64bit3=0;
	for(int i=0;i<param_data_num;i++){
		data64bit3|=(uint64_t)param_data[i]<<(8*i);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr+(MAZESIZE_X*MAZESIZE_Y), data64bit3);

	HAL_FLASH_Lock();
}

void FlashSetGoalFlag(bool flag){
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_addr+(MAZESIZE_Y*MAZESIZE_X)+8+8, flag);
	HAL_FLASH_Lock();
}

uint8_t FlashGetGoalFlag(){
	uint8_t data=flash_data[MAZESIZE_Y*MAZESIZE_X];
	return data;
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

