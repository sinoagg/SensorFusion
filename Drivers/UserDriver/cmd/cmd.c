#include "cmd.h"

uint8_t GetRadarData(uint8_t warning_state, uint8_t vehicle_speed, float dist, float crashing_time)
{
	RadarData.Warning_State = warning_state;
	RadarData.Vehicle_Speed = vehicle_speed;
	RadarData.Dist.f = dist;
	RadarData.Crash_Time.f = crashing_time;
	return 0;
}

void FillRadarDataTxBuf(uint8_t * pCmdRadarData, Cmd_RadarData RadarData)
{
	uint8_t i = 0;
	*pCmdRadarData = RadarData.Sys_State;
	*(pCmdRadarData + 1) = RadarData.Warning_State;
	*(pCmdRadarData + 2) = RadarData.Vehicle_Speed;
	for(i = 0; i < 4; i++)
	{
		*(pCmdRadarData + 3 + i) = RadarData.Dist.Arr[i];
		*(pCmdRadarData + 7 + i) = RadarData.Crash_Time.Arr[i];
	}
}

