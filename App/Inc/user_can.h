#ifndef __USER_CAN_H
#define __USER_CAN_H

#include "stm32f4xx_hal.h"
#include "can.h"
#include "vehicle.h"

#define DBC_ADDR  0x509 //	can1 id, dbc

extern uint8_t AEB_CAN_TxReady;
extern uint8_t VehicleCANRxBuf[];
extern uint8_t RadarCANRxBuf[];
extern uint8_t YawCANRxBuf[];
extern uint8_t AEB_CAN_TxBuf[];
extern uint32_t AEB_CAN_TxMailBox;
extern uint32_t DBC_CAN_TxMailBox;
extern CAN_RxHeaderTypeDef VehicleCANRxHeader;
extern CAN_RxHeaderTypeDef RadarCAN_RxHeader;
extern CAN_TxHeaderTypeDef DBC_CAN_TxHeader;
extern CAN_TxHeaderTypeDef AEB_CAN_TxHeader;

uint8_t Vehicle_CAN_Init(CAN_HandleTypeDef *hcan);
uint8_t CAN1_Init(CAN_HandleTypeDef * hcan);	

#endif

