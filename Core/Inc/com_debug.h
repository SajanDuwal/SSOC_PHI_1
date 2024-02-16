/*
 * com_debug.h
 *
 *  Created on: Oct 21, 2023
 *      Author: sajanduwal
 */

#ifndef INC_COM_DEBUG_H_
#define INC_COM_DEBUG_H_

#include "main.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

void myPrintf(const char *fmt, ...);

int bufferSize(char *buffer);

void delay_us(uint16_t ms);

#endif /* INC_COM_DEBUG_H_ */
