#ifndef __MPU6050_H
#define __MPU6050_H
/**
 * Gyro - MPU6050
 * ===functions:
 * MPU_GetYawRate(CANRxBuf): get YawRate of gyro
 */

#include "stm32f4xx_hal.h"

float MPU_GetYawRate(uint8_t *pRxBuf);

#endif
