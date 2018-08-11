#ifndef __cmd_H
#define __cmd_H

#include "stm32f4xx_hal.h"

#define RADAR_OK 0x00
#define RADAR_ERROR 0x01
typedef union
{
	float f;
	uint8_t Arr[4];
}Float2Arr;

/**
 *  Sys_State
 * 		0x00 OK
 * 		0x01 ERROR
 * 	Warning_State(defined in main.c)
 * 		0x00 WARNING_NONE
 * 		0x01 WARNING_LOW
 * 		0x02 WARNING_MID
 * 		0x03 WARNING_HIGH
 * 	Vehicle_Speed
 * 		km/h
 * 		offset 0
 * 		range 0~255
 * 	Dist
 * 		Array[4] of float, low to high
 * 	Crash_time
 * 		Array[4] of float, low to high
 */
typedef struct
{
	uint8_t Sys_State;
	uint8_t Warning_State;
	uint8_t Vehicle_Speed;
	Float2Arr Dist;
	Float2Arr Crash_Time;
}Cmd_RadarData;

extern uint8_t CmdRadarDataBuf[];
extern Cmd_RadarData RadarData;

uint8_t GetRadarData(uint8_t warning_state, uint8_t vehicle_speed, float dist, float crashing_time);
void FillRadarDataTxBuf(uint8_t * pCmdRadarData, Cmd_RadarData RadarData);

#endif
