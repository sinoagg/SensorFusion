/**
 * Gyro - MPU6050
 * 
 * MPU_***() functions are entries of this driver
 * ***_func() functions are practical functions called by MPU_***() functions
 * 
 * MPU_GetYawRate(CANRxBuf): get YawRate of gyro
 */
#include "MPU6050.h"

/** 
 * @brief  get YawRate from can3 buf
 * @note   resolution is 0.2°/s
 * @param  *pRxBuf: CANRxBuf
 * @retval YawRate(float)
 */
float MPU_GetYawRate(uint8_t *pRxBuf)
{
	float YawRate = 0.0;

  YawRate = (*(pRxBuf + 6))*0.2 - 25;	//offset -25°/s, Res 0.2°/s
	
	return YawRate;
}

