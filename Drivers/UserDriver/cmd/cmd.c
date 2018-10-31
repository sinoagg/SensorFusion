/** 
 * @brief  send RadarData to labview
 * ! not using
 * @param  warning_state: crash warning(high/low)
 * @param  vehicle_speed: km/h(read from ARS4081)
 * @param  dist: m(read from ARS408)
 * @param  crashing_time: calculate by ARS408.c
 * * sending data via RS232 
 */
#include "cmd.h"

Cmd_RadarData RadarData;
/** 
 * @brief  get Radar Data
 * @note   assign valuse to RadarData struct
 * @param  warning_state: 
 * @param  vehicle_speed: (km/h)
 * @param  dist: (m)
 * @param  crashing_time: (s)
 * @retval 0 for ok
 */
uint8_t GetRadarData(uint8_t warning_state, uint8_t vehicle_speed, float dist, float crashing_time)
{
	RadarData.Warning_State = warning_state;
	RadarData.Vehicle_Speed = vehicle_speed;
	RadarData.Dist.f = dist;
	RadarData.Crash_Time.f = crashing_time;
	return 0;
}

/** 
 * @brief  Fill TxBuf
 * @note   RadarData struct assign values to TxBuf
 * @param  pCmdRadarData: TxBuf
 * @param  RadarData: struct
 * @retval None
 */
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

