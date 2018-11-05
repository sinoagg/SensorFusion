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
 * @note   resolution is 0.2 бу/s
 * @param  *pRxBuf: CANRxBuf
 * @retval YawRate(float)
 */
float MPU_GetYawRate(uint8_t *pRxBuf)
{
	float YawRate = 0.0;

//  YawRate = (*(pRxBuf + 6))*0.2 - 25;	//offset -25 бу/s, Res 0.2 бу/s
	YawRate = (((*(pRxBuf + 4))<<8) | (*(pRxBuf + 3))) / 32768.0f * 2000;
	if(YawRate > 2000)
		YawRate -= 4000;
	
	if(YawRate < 0)		//clockwise
		{
			YawRate = (YawRate < -YAWRATE_LIMIT) ? -YAWRATE_LIMIT : YawRate;
		}
		else
			YawRate = (YawRate > YAWRATE_LIMIT) ? YAWRATE_LIMIT: YawRate;
	
	return YawRate;
}

/** 
 * @brief  get Xacceleration from can3 buf
 * @note   resolution is 0.02 g/bit
 * @param  *pRxBuf: CANRxBuf
 * @retval XAcceleration(float)Buf
 */
float MPU_GetXAcc(uint8_t *pRxBuf)
{
	float XAcc = 0.0;

//	XAcc = ((*pRxBuf +1)) * 0.02 - 2.5; //offset -2.5, Res 0.02
  XAcc = ((*(pRxBuf + 2))<<8 | (*(pRxBuf + 1))) /32768.0f * 16;	//offset -2.5 g/bit, Res 0.02 g/bit
//	if(XAcc<16)
//		XAcc = (XAcc < 2.5f) ? XAcc : 2.5f;
//	else
//		XAcc = ((XAcc > 2.5f) ? XAcc : 2.5f) - 32;
	
		if(XAcc > 16)
			XAcc -= 32;
	XAcc *= 9.8f;	//m/s2
	
	return XAcc;
}
