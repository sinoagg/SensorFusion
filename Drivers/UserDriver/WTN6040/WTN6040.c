#include "WTN6040.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "delay.h"
#include "main.h"

/*实现二线串口通信函数*/

uint8_t WTN6_Broadcast(uint8_t addr)
{
	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(BELL_BUSY_GPIO_Port, BELL_BUSY_Pin))
	{
		uint8_t bit_data;
		uint8_t j;
		HAL_GPIO_WritePin(BELL_CLK_GPIO_Port, BELL_CLK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BELL_CLK_GPIO_Port, BELL_CLK_Pin, GPIO_PIN_RESET);
		delay_ms(5);
		for (j = 0; j < 8; j++)
		{
			bit_data = addr & 0X01;
			HAL_GPIO_WritePin(BELL_CLK_GPIO_Port, BELL_CLK_Pin, GPIO_PIN_RESET);
			if (bit_data == 1)
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_RESET);
			delay_us(300);
			HAL_GPIO_WritePin(BELL_CLK_GPIO_Port, BELL_CLK_Pin, GPIO_PIN_SET);
			delay_us(300);
			addr = addr >> 1;
		}
		HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BELL_CLK_GPIO_Port, BELL_CLK_Pin, GPIO_PIN_SET);
		return 0;
	}
	else
		return 1;
}

/*实现一线串口通信函数*/
/*
uint8_t WTN6_Broadcast(uint8_t addr)
{
	if(GPIO_PIN_SET==HAL_GPIO_ReadPin( BELL_BUSY_GPIO_Port,  BELL_BUSY_Pin))
	{
		uint8_t bit_data;
		uint8_t j;
		HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_RESET);
		//osDelay(5);
		delay_ms(5); 
		for(j=0;j<8;j++)
		{
			bit_data = addr&0X01;
			if(bit_data == 1)
			{
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
				delay_us(600); 
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_RESET);
				delay_us(200);
			}
			else
			{
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
				delay_us(200);
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_RESET);
				delay_us(600); 
			}
			addr = addr>>1;
		}
		HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
		return 0;
	}
	else 
		return 1;
}
*/
