#ifndef __VEHICLE_H
#define __VEHICLE_H

#include "stm32f4xx_hal.h"

#define KINGLONG 0
#define YUTONG 1
#define BYD 2
#define BENZ 3

#define BENZ_VEHICLE_SPEED_ADDR 0x18FE6C00

#define BYD_VEHICLE_SPEED_ADDR	0x18FEF100

#define YUTONG_VEHICLE_SPEED_ADDR	0x18FE6E0B
	
#define KINGLONG_VEHICLE_SPEED_ADDR		0x0CFE6CEE
#define KINGLONG_VEHICLE_SWITCH_ADDR	0x18FA0517
#define KINGLONG_VEHICLE_ANGLE_ADDR		0x18F0090B
#define KINGLONG_VEHICLE_AEBS_ADDR   	0x0CF02FA0

#define VEHICLE_MODEL KINGLONG
#define VEHICLE_SPEED_ADDR 	KINGLONG_VEHICLE_SPEED_ADDR
#define VEHICLE_SWITCH_ADDR KINGLONG_VEHICLE_SWITCH_ADDR
#define VEHICLE_ANGLE_ADDR 	KINGLONG_VEHICLE_ANGLE_ADDR
#define VEHICLE_AEBS_ADDR		KINGLONG_VEHICLE_AEBS_ADDR

typedef struct
{
  uint8_t brake;
  uint8_t right_turn;
  uint8_t left_turn;
}VehicleSwitchTypeDef;

typedef struct
{
	float tw_angle;	//turn wheel
	uint8_t  tw_circle;	
	uint8_t  tw_type;
	float yawRate;
	float latAcc;	//latitude
	float  longAcc;	//longitude
	uint8_t speed;
}VehicleTypeDef;

extern VehicleSwitchTypeDef vehicleSwitch;
extern VehicleTypeDef vehicle;

#endif

