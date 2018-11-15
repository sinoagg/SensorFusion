#include "user_adas.h"

ADAS_HandleTypeDef ADAS_dev;
uint8_t ADASRxComplete = 0;
uint8_t ADASRxBuf[UART3BUFSIZE*2] = {0};
uint8_t ADASHexBuf[UART3BUFSIZE] = {0};
uint8_t ADASDispBuf[32] = {0};

void Ascii2Hex(uint8_t size, uint8_t *AsciiBuf, uint8_t *HexBuf)
{
	uint8_t i;
	for(i=0;i<size;i++)
	{
		if(AsciiBuf[2*i]<0x3a)
			HexBuf[i]=(AsciiBuf[2*i]-0x30)<<4;
		else
			HexBuf[i]=(AsciiBuf[2*i]-0x37)<<4;
		if(AsciiBuf[2*i+1]<0x3a)
			HexBuf[i]=((AsciiBuf[2*i+1]-0x30)& 0x0F)|HexBuf[i];
		else
			HexBuf[i]=((AsciiBuf[2*i+1]-0x37)& 0x0F)|HexBuf[i];
	}
}

void Hex2Ascii(uint8_t size, uint8_t *HexBuf, uint8_t *AsciiBuf)
{
	uint8_t i;
	for(i=0;i<size;i++)
	{
		if((HexBuf[i]>>4&0x0f)<10)
		{
			AsciiBuf[2*i]=(HexBuf[i]>>4&0x0f)+0x30;
		}
		else
		{
			AsciiBuf[2*i]=(HexBuf[i]>>4&0x0f)+0x37;
		}
		if((HexBuf[i]&0x0f)<10)
		{
			AsciiBuf[2*i+1]=(HexBuf[i]&0x0f)+0x30;
		}
		else
		{
			AsciiBuf[2*i+1]=(HexBuf[i]&0x0f)+0x37;
		}
  //AsciiBuf[2*i+1]=(HexBuf[i]&0x0f)+0x30;
	}
}
