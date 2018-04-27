#ifndef __DELAY_H
#define __DELAY_H	 

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"	
	 
void delay_init(uint8_t SYSCLK);	
void Delay_us(uint32_t nus);
void Delay_ms(uint32_t ms);	 
#endif

