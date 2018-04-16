#ifndef __ADAS_H
#define __ADAS_H

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "WTN6040.h"

typedef struct
{
	uint8_t DEV_work_status;																//整体设备状态
	uint8_t LDW_work_status;																//LDW工作状态
	uint8_t FCW_work_status;																//FCW工作状态
	uint8_t PCW_work_status;																//PCW工作状态
	uint8_t DFW_work_status;																//DFW工作状态
	uint8_t LDW_warning;																		//LDW车道偏离 0x00正常，0x01左偏离 0x02右偏离 0x03保留
	uint8_t crash_type;																			//0x00 FCW ，0x01 PCW
	uint8_t crash_level;																		//0x00正常 0x01一级预警 0x02二级预警 0x03三级预警
	uint8_t driver_status;																	//0x000正常 0x001疲劳 0x002姿态异常 0x003打电话 0x004抽烟
	uint8_t	self_check_main;																//自检状态
	uint8_t line_type;																			//车道线类型
	uint8_t distance_left_line;															//分辨率0.125
	uint8_t distance_right_line;														//分辨率0.125
	uint8_t line_circle_diameter;														//车道线曲率半径 分辨率4
	uint8_t line_width;
	uint8_t direction_angle;																//分辨率1，偏移-90
	uint8_t distance_to_target; 														//分辨率0.5
	uint8_t speed_to_target;																//分辨率0.25
	float time_to_target;																		//分辨率0.05
	
} ADAS_HandleTypeDef;

uint8_t CalADASData(ADAS_HandleTypeDef *pADAS_dev, uint8_t *pRxBuf);

#endif

