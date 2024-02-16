/*
 * rtc.h
 *
 *  Created on: Oct 22, 2023
 *      Author: sajanduwal
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "main.h"

extern RTC_HandleTypeDef hrtc;

extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

extern RTC_TimeTypeDef gTime;
extern RTC_DateTypeDef gDate;

void setTime(uint8_t year, uint8_t month, uint8_t weekDay, uint8_t hour,
		uint8_t min, uint8_t sec);

void getTime();

#endif /* INC_RTC_H_ */
