#ifndef __EMRR_H
#define __EMRR_H
/**
 * microwave radar - EMRR by Electronic Radar(WuHu) 
 * ===functions:
 * ---init & config
 * EMRR_Init(hcan): start CAN2 for radar & (optional) config Radadr and RadarFilter
 * 
 * ---read Radar data
 * EMRR_GetRadarObjGeneral(...): read Radar Obj General(distance, relVelocity...) from can2
 * ===#defines:
 * ADDR: (can id, std)
 * (write)config addr 0x200 ~ 0x408
 * (read) obj & cluster addr 0x600~0x60E
 * 
 * VALID && config value: values with shift(<<)
 * RADAR
 * FILTER
 *
 * structs
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"
#include"math.h"

typedef struct
{
	uint32_t    trackId;
	float    trackRange;
  float    trackSpeed;
  float    trackAngle;
  float    trackPower;
	float    trackCrossRange;
}MW_RadarGeneral;

#endif
