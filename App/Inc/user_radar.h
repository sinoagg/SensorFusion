#ifndef __USER_RADAR_H
#define __USER_RADAR_H

#include "stm32f4xx_hal.h"


typedef struct 
{
	float VrelLong;
	float MinRangeLong;
	float VrelLat;
	float MinRangeLat;
}ObjectTypeDef;

#if RADAR_TYPE == ARS408
	#include "ARS408.h"
  extern MW_RadarObjStatus RadarObjStatus;
	extern MW_RadarGeneral RadarGeneral[];
#elif RADAR_TYPE == EMRR
	#include "EMRR.h"
  extern EMRR_RadarGeneral aEMRRGeneral[];
	extern EMRR_RadarGeneral EMRRGeneral_Closet;
	extern uint8_t EMRR_RadarRxComplete;
	extern uint8_t EMRR_RadarObjCount;
#endif

extern ObjectTypeDef RadarObject;

#endif
