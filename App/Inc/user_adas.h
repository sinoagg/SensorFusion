#ifndef __USER_ADAS_H
#define __USER_ADAS_H

#include "stm32f4xx_hal.h"
#include "ADAS.h"

#define UART3BUFSIZE 32

extern ADAS_HandleTypeDef ADAS_dev;
extern uint8_t ADASRxComplete;
extern uint8_t ADASRxBuf[UART3BUFSIZE];
extern uint8_t ADASDispBuf[32];

#endif
