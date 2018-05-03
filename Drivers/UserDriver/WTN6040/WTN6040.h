#ifndef __WTN6040_H
#define __WTN6040_H
#include "stm32f4xx_hal.h"

//bell ring address
#define BELL_BB_1000MS 0x15
#define BELL_BB_500MS 0x16
#define BELL_BB_300MS 0x17
#define BELL_BB_200MS 0x18

#define BELL_BIRD_1000MS 0x19
#define BELL_BIRD_500MS 0x1A
#define BELL_BIRD_300MS 0x1B
#define BELL_BIRD_200MS 0x1C

#define BELL_WHILE 0xF2
#define BELL_STOP 0xFE

uint8_t WTN6_Broadcast(uint8_t addr);
		
#endif
