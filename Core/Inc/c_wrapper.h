/*
 * c_wrapper.h
 *
 *  Created on: Oct 18, 2023
 *      Author: Makihara
 */

#ifndef INC_C_WRAPPER_H_
#define INC_C_WRAPPER_H_

#if __cplusplus
extern "C"{
#endif

void Init();

void Loop();

void Interrupt1ms();
void Interrupt100us();

#if __cplusplus
};
#endif


#endif /* INC_C_WRAPPER_H_ */
