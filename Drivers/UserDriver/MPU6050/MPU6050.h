#ifndef __MPU6050_H
#define __MPU6050_H
/**
 * Gyro - MPU6050
 * ===functions:
 * MPU_GetYawRate(CANRxBuf): get YawRate of gyro
 * MPU_GetXAcc(CANRxBuf): get XAcceleration from gyro
 */

#include "stm32f4xx_hal.h"

float MPU_GetYawRate(uint8_t *pRxBuf);
float MPU_GetXAcc(uint8_t *pRxBuf);

#endif
