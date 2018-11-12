#ifndef __AEBS_H
#define __AEBS_H

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"

#define LOW_WARNING_TIM_PERIOD 200 //ms
#define HIGH_WARNING_TIM_PERIOD 50 //ms

#define VEHICLE_SPEED_THRESHOLD 0

#define LOW_WARNING_TIME 5.4f
#define MID_WARNING_TIME 4.8f
#define HIGH_WARNING_TIME 4.5f

#define ADAS_COMM 1

#define LIGHT_TIME 3.8f
#define SOUND_TIME 4.4f

#define ARS408 1
#define EMRR 0
/*------Radar type------*/
#define RADAR_TYPE ARS408

#define LANEWIDTH 1.5f
#define MAX_DECELARATION 0.4 * 9.8f
#define DELAY_TIME 0.4f
#define LIMIT_RANGE 200 //À×´ïÌ½²â·¶Î§

#define CAN_READ_VEHICLE 1 //Vehicle Speed & Switch
#define AEB_CAN_TX_TIME 50 //CAN Bus 50ms interval

#define WARNING_NONE 0	//3
#define WARNING_LOW 1		//5 
#define WARNING_MID 4		//6
#define WARNING_HIGH 7	//7

#define BRAKE_SYS_RDY		0x03
#define COLLISION_WARNING_ACTIVE  0x05
#define BRAKE_SYS_ON	0x06
#define BRAKE_SYS_EMER	0x07

#define OBJECT_NOT_DETECTED 0
#define OBJECT_DETECTED 1

#define INVADE_LANE_TIME_THRESHOLD 5.0f

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
extern uint8_t crashWarningLv;
extern AEBS_Status vAEBS_Status;
extern CAN_HandleTypeDef hcan2;

void StopBuzzer(AEBS_Status *pAEBS_Status);
void StartBuzzer(AEBS_Status *pAEBS_Status, uint8_t warningLv);
void DisableAEBS(AEBS_Status *pAEBS_Status);
void EnableAEBS(float ttc, uint8_t warningLv);
uint8_t ValveCalc(DAC_HandleTypeDef *hdac, float ttc);
uint8_t XBRCalc(CAN_HandleTypeDef *hcan, float ttc, uint8_t XBR_Ctrl);
uint8_t PrePareAEBS1Data(CAN_HandleTypeDef *hcan, uint8_t brakeSysState, uint8_t warningLv, uint8_t objectDetected);

#endif
