#include "user_radar.h"
#include "can.h"
//#include "aebs.h"

ObjectTypeDef RadarObject;

#if RADAR_TYPE == ARS408
	#include "ARS408.h"
  MW_RadarObjStatus RadarObjStatus;
	MW_RadarGeneral RadarGeneral[16];
#elif RADAR_TYPE == EMRR
	#include "EMRR.h"
  EMRR_RadarGeneral aEMRRGeneral[64];
	EMRR_RadarGeneral EMRRGeneral_Closet;
#endif
