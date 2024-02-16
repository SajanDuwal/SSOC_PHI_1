/*
 * obc_interface.c
 *
 *  Created on: Oct 21, 2023
 *      Author: sajanduwal
 */

#include "obc_interface.h"
#include "main.h"
#include "com_debug.h"
#include "string.h"

uint8_t MAIN_CMD[15];
uint8_t MCU_ID;

uint32_t MSN_CMD;
uint32_t MAIN_ADDR;

void WAIT_FOR_HANDSHAKE() {
	uint8_t MainCMDHs[5];
	if (HAL_UART_Receive(&huart1, MainCMDHs, 5, 7000) == HAL_OK) {
		myPrintf("--> HandShake command received: 0x");
		for (int i = 0; i < 5; i++) {
			myPrintf("%x", MainCMDHs[i]);
		}
		myPrintf("\n");
		delay_us(1);
		if (MainCMDHs[0] == 0x04 && MainCMDHs[4] == 0xFE) {
			myPrintf("--> Command Acknowledged!\n");
			if (HAL_UART_Transmit(&huart1, MainCMDHs, 5, 2000) == HAL_OK) {
				myPrintf("--> HandShake ACK sent to MAIN\n");
				delay_us(1);
			}
		} else {
			myPrintf("*** Unknown handshake command received\n");
			delay_us(1);
			WAIT_FOR_HANDSHAKE();
		}
	} else {
		delay_us(1);
		WAIT_FOR_HANDSHAKE();
	}
}

void Receive_MAIN_CMD() {
	if (HAL_UART_Receive(&huart1, MAIN_CMD, 15, 7000) == HAL_OK) {
		myPrintf("--> Command received from OBC: 0x");
		for (int i = 0; i < 15; i++) {
			myPrintf("%x\r", MAIN_CMD[i]);
		}
		myPrintf("\n");
		delay_us(1);
		if (MAIN_CMD[0] == 0x01 && MAIN_CMD[14] == 0xFE) {
			myPrintf("--> Correct command received from OBC\n");
		} else {
			myPrintf("*** Incorrect command received from OBC\n");
			delay_us(1);
			Receive_MAIN_CMD();
		}
	} else {
		myPrintf("*** Command receive failed\n");
		delay_us(1);
		Receive_MAIN_CMD();
	}
}

void Execute_MAIN_CMD() {
	myPrintf("### Fetching received CMD from OBC.....\n");
	MCU_ID = MAIN_CMD[0];
	MSN_CMD = MAIN_CMD[0] << 24 | MAIN_CMD[1] << 16 | MAIN_CMD[2] << 8
			| MAIN_CMD[3];
	MAIN_ADDR = MAIN_CMD[4] << 24 | MAIN_CMD[5] << 16 | MAIN_CMD[6] << 8
			| MAIN_CMD[7];
}
