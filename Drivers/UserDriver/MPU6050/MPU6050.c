/**
 * Gyro - MPU6050
 * 
 * MPU_***() functions are entries of this driver
 * ***_func() functions are practical functions called by MPU_***() functions
 * 
 * MPU_CheckSum(CANRxBuf): CheckSum of CAN received buffer
 * MPU_GetYawRate(CANRxBuf): get YawRate of gyro
 * MPU_GetYaw(CANRxBuf): get Yaw of gyro
 */
#include "MPU6050.h"

uint8_t MPU_CheckSum(uint8_t *pRxBuf)
{
	uint8_t i = 0, sum = 0;

	for(; i < 5; i ++)
		sum += *(pRxBuf + i);

	if(*(pRxBuf + 5) == sum)
		return 1;
	else
		return 0;
}

float MPU_GetYawRate(uint8_t *pRxBuf)
{
	float YawRate = 0.0;
	uint8_t YawRateL = 0, YawRateH = 0;

  YawRateL = *(pRxBuf + 3);
  YawRateH = *(pRxBuf + 4);
	YawRate = ((float)((YawRateH<<8) | YawRateL)) / 32768.0f * 2000;  //单位是°/s
	
	return YawRate;
}

float MPU_GetYaw(uint8_t *pRxBuf)
{
	float Yaw = 0.0;
	uint8_t YawL = 0, YawH = 0;

  YawL = *(pRxBuf + 1);
  YawH = *(pRxBuf + 2);
  Yaw = ((float)((YawH<<8) | YawL)) / 32768.0f * 180;  //单位是°

	return Yaw;
}

