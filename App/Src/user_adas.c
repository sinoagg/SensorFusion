#include "user_adas.h"

ADAS_HandleTypeDef ADAS_dev;
uint8_t ADASRxComplete=0;
uint8_t ADASRxBuf[UART3BUFSIZE]={0};
uint8_t ADASDispBuf[32] = {0};
