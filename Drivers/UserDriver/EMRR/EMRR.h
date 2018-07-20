#ifndef __EMRR_H
#define __EMRR_H
/**
 * microwave radar - EMRR by Electronic Radar(WuHu) 
 * ===functions:
 * ---init & config
 * EMRR_Init(hcan): start CAN2 for radar
 * ---read Radar data
 * EMRR_GetRadarObjCloset(...): get closet Radar Obj General(distance, relVelocity...) from can2
 *
 * ===#defines:
 * ADDR: (can id, std)
 * (read) obj addr 0x500~0x53F
 * 
 * ===struct:
 * EMRR_RadarGeneral
 */

#include "stm32f4xx_hal.h"
#include"math.h"

/* µÿ÷∑∂®“Â--------------------------------------*/
#define EMRR_OBJ_ADDR 0x500


typedef struct
{
	uint32_t    trackId;
	float    trackRange;
  float    trackSpeed;
  float    trackAngle;
  float    trackPower;
	float    trackCrossRange;
}EMRR_RadarGeneral;


uint8_t EMRR_Init(CAN_HandleTypeDef *hcan);
void EMRR_GetRaderObjCloset(uint8_t* pCANRxBuf, EMRR_RadarGeneral *pRadarGeneral, EMRR_RadarGeneral *pRadarGeneral_closet);

#endif
