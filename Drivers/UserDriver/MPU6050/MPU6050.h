#ifndef __MPU6050_H
#define __MPU6050_H
/**
 * Gyro - MPU6050
 * ===functions:
 * MPU_CheckSum(CANRxBuf): CheckSum of CAN received buffer
 * MPU_GetYawRate(CANRxBuf): get YawRate of gyro
 * MPU_GetYaw(CANRxBuf): get Yaw of gyro
 */

#include "stm32f4xx_hal.h"

uint8_t MPU_CheckSum(uint8_t *pRxBuf);
float MPU_GetYawRate(uint8_t *pRxBuf);
float MPU_GetYaw(uint8_t *pRxBuf);

#endif
