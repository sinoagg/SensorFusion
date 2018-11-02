#ifndef __AEBS_H
#define __AEBS_H

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"

#define LOW_WARNING_TIM_PERIOD 200 //ms
#define HIGH_WARNING_TIM_PERIOD 50 //ms

#define VEHICLE_SPEED_THRESHOLD 5

#define LOW_WARNING_TIME 4.4f
#define MID_WARNING_TIME 3.8f
#define HIGH_WARNING_TIME 3.0f

#define ADAS_COMM 1

#define LIGHT_TIME 3.8f
#define SOUND_TIME 4.4f

#define ARS408 1
#define EMRR 0
#define RADAR_TYPE ARS408

#if RADAR_TYPE == ARS408
	#define RADAR_OFFSET 0.4f
#elif RADAR_TYPE == EMRR
	#define RADAR_OFFSET 0.0f
#endif

#define LANEWIDTH 1.5f
#define MAX_DECELARATION 0.4 * 9.8f
#define DELAY_TIME 0.4f
#define LIMIT_RANGE 200 //À×´ïÌ½²â·¶Î§

#define CAN_READ_VEHICLE 1 //Vehicle Speed & Switch
#define AEB_CAN_TX_TIME 50 //CAN Bus 50ms interval

#define WARNING_NONE 3
#define WARNING_LOW 5
#define WARNING_MID 6
#define WARNING_HIGH 7

#define ON 1
#define OFF 0

/* ---Turning_Flag values---*/
#define TURNING 1
#define STRAIGHT 0
/* ---Turning_Collision flag values---*/
#define TURNING_COLLISION_WARNING 1
#define TURNING_COLLISION_NONE 0


typedef struct
{
	uint8_t BuzzerStatus;
	uint8_t valveStatus;
} AEBS_Status;

extern float TimetoCrash_g;
extern uint8_t CrashWarningLv;
extern AEBS_Status vAEBS_Status;

void StopBuzzer(AEBS_Status *pAEBS_Status);
void StartBuzzer(uint8_t warningLv);
void DisableAEBS(AEBS_Status *pAEBS_Status);
void EnableAEBS(float ttc, uint8_t warningLv);
uint8_t ValveCalc(DAC_HandleTypeDef *hdac, float ttc);

#endif
