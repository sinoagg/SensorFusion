#ifndef __USER_ADAS_H
#define __USER_ADAS_H

#include "stm32f4xx_hal.h"
#include "ADAS.h"

#define UART3BUFSIZE 32

extern ADAS_HandleTypeDef ADAS_dev;
extern uint8_t ADASRxComplete;
extern uint8_t ADASRxBuf[UART3BUFSIZE*2];
extern uint8_t ADASHexBuf[UART3BUFSIZE];
extern uint8_t ADASDispBuf[32];

void Ascii2Hex(uint8_t size, uint8_t *AsciiBuf, uint8_t *HexBuf);
void Hex2Ascii(uint8_t size, uint8_t *HexBuf, uint8_t *AsciiBuf);

#endif
