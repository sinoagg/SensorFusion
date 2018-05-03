#include "WTN6040.h"
#include "stm32f4xx_hal.h"
#include "delay.h"
#include "main.h"

/*实现一线串口通信函数*/

uint8_t WTN6_Broadcast(uint8_t addr)
{
	if(GPIO_PIN_SET==HAL_GPIO_ReadPin( BELL_BUSY_GPIO_Port,  BELL_BUSY_Pin))
	{
		uint8_t bit_data;
		uint8_t j;
		HAL_GPIO_WritePin(GPIOD, BELL_DATA_Pin, GPIO_PIN_RESET);
		Delay_ms(5); 
		for(j=0;j<8;j++)
		{
			bit_data = addr&0X01;
			if(bit_data == 1)
			{
				HAL_GPIO_WritePin(GPIOD, BELL_DATA_Pin, GPIO_PIN_SET);
				Delay_us(600); 
				HAL_GPIO_WritePin(GPIOD, BELL_DATA_Pin, GPIO_PIN_RESET);
				Delay_us(200);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD, BELL_DATA_Pin, GPIO_PIN_SET);
				Delay_us(200);
				HAL_GPIO_WritePin(GPIOD, BELL_DATA_Pin, GPIO_PIN_RESET);
				Delay_us(600); 
			}
			addr = addr>>1;
		}
		HAL_GPIO_WritePin(GPIOD, BELL_DATA_Pin, GPIO_PIN_SET);
		return 0;
	}
	else 
		return 1;
}


/*实现二线串口通信函数(未测试)*/			
/*
void WTN6040_Send(uint8_t WTN6040_DATA)                    			
{
	uint8_t i=0;
	uint8_t	bit=0;
	HAL_GPIO_WritePin(GPIOB, BELL_DATA_Pin, GPIO_PIN_SET);				//时钟线
	HAL_GPIO_WritePin(GPIOA, bell_data_Pin, GPIO_PIN_SET);				//数据线
	HAL_GPIO_WritePin(GPIOB, BELL_DATA_Pin, GPIO_PIN_RESET);
	Delay_ms(5);
	bit=WTN6040_DATA&0x01;
	for(i=0;i<8;i++)
	{
		bit=WTN6040_DATA&0x01;
		HAL_GPIO_WritePin(GPIOB, BELL_DATA_Pin, GPIO_PIN_RESET);;
		if(bit==1)
			HAL_GPIO_WritePin(GPIOA, bell_data_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOA, bell_data_Pin, GPIO_PIN_RESET);
		Delay_us(300);
		HAL_GPIO_WritePin(GPIOB, BELL_DATA_Pin, GPIO_PIN_SET);
		Delay_us(300);
		WTN6040_DATA=WTN6040_DATA>>1;
	}
	HAL_GPIO_WritePin(GPIOA, bell_data_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, BELL_DATA_Pin, GPIO_PIN_SET);
}
*/

