#ifndef FLASH_UTIL_HPP_
#define FLASH_UTIL_HPP_

#include "MazeDef.hpp"
#include "stdint.h"

void FlashInit();
uint32_t FlashEraseData();
void FlashGetMazeData(int data[MAZESIZE_X][MAZESIZE_Y]);
void FlashSetMazeData(int data[MAZESIZE_X][MAZESIZE_Y]);
void FlashPrintMazeData(int data[MAZESIZE_X][MAZESIZE_Y]);

#endif /* FLASH_UTIL_HPP_ */
