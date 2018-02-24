#ifndef __SRC_SLEEP_H__
#define __SRC_SLEEP_H__

#include "stm32l0xx_hal.h"

extern RTC_HandleTypeDef hrtc;

void Sleep(int sleepSec);


#endif
