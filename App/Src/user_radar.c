#include "user_radar.h"
#include "can.h"

ObjectTypeDef RadarObject;

#if RADAR_TYPE == ARS408
	#include "ARS408.h"
  MW_RadarObjStatus RadarObjStatus;
	MW_RadarGeneral RadarGeneral[16];
	uint8_t MW_RadarRxComplete=0;
	uint8_t CmdRxComplete=0;
	uint8_t CmdRxBuf[4]={0};
	uint8_t CmdRadarDataTxBuf[11];
#elif RADAR_TYPE == EMRR
	#include "EMRR.h"
  EMRR_RadarGeneral aEMRRGeneral[64];
	EMRR_RadarGeneral EMRRGeneral_Closet;
	uint8_t EMRR_RadarRxComplete=0;
	uint8_t EMRR_RadarObjCount = 0;
#endif
