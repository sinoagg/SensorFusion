#include "user_can.h"
#include "vehicle.h"

uint8_t VehicleCANRxBuf[8]={0};
uint8_t RadarCANRxBuf[8]={0};
uint8_t YawCANRxBuf[8] = {0};
uint8_t AEB_CAN_TxBuf[8];
uint8_t AEB_CAN_TxReady=0;

uint32_t AEB_CAN_TxMailBox = CAN_TX_MAILBOX0;
uint32_t DBC_CAN_TxMailBox = CAN_TX_MAILBOX0;
CAN_RxHeaderTypeDef VehicleCANRxHeader;
CAN_RxHeaderTypeDef RadarCAN_RxHeader;
CAN_TxHeaderTypeDef AEB_CAN_TxHeader={0, VEHICLE_AEBS_ADDR, CAN_ID_EXT, CAN_RTR_DATA, 8, DISABLE};
CAN_TxHeaderTypeDef DBC_CAN_TxHeader={DBC_ADDR, 0, CAN_ID_EXT, CAN_RTR_DATA, 8, DISABLE};

uint8_t Vehicle_CAN_Init(CAN_HandleTypeDef * hcan)
{
	//config CAN3 filter to receive Vehicle Speed
	//ID_HIGH,ID_LOW,MASK_HIGH,MASK_LOW,FIFO 0/1, filter_bank(0-13/14-27), filter_mode(LIST/MASK), filter_scale, EN/DISABLE filter, SlaveStartFilterBank
	CAN_FilterTypeDef VehicleCANFilter = {
		VEHICLE_SPEED_ADDR>>13 & 0xFFFF,\
		((VEHICLE_SPEED_ADDR & 0xFFFF) <<3) | 0x4,\
		0xFF<<3 | 0xF,\
		0xFF00<<3,\
		CAN_FILTER_FIFO1, 14, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,14
	};
	HAL_CAN_ConfigFilter(hcan, &VehicleCANFilter);
	
	//config CAN3 filter to receive Vehicle switch data
	//ID_HIGH,ID_LOW,MASK_HIGH,MASK_LOW,FIFO 0/1, filter_bank(0-13/14-27), filter_mode(LIST/MASK), filter_scale, EN/DISABLE filter, SlaveStartFilterBank
  //#if VEHICLE_MODEL == KINGLONG
	/*CAN_FilterTypeDef VehicleSwitchCANFilter = {
		VEHICLE_SWITCH_ADDR>>13 & 0xFFFF,\
		((VEHICLE_SWITCH_ADDR & 0xFFFF) <<3) | 0x4,\
		0xFF<<3 | 0xF,\
		0xFF00<<3,\
		CAN_FILTER_FIFO1, 15, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,14
	};
	HAL_CAN_ConfigFilter(hcan, &VehicleSwitchCANFilter);
	
	//ID_HIGH,ID_LOW,MASK_HIGH,MASK_LOW,FIFO 0/1, filter_bank(0-13/14-27), filter_mode(LIST/MASK), filter_scale, EN/DISABLE filter, SlaveStartFilterBank
	*/
	/*8CAN_FilterTypeDef VehicleAngleCANFilter = {
		VEHICLE_ANGLE_ADDR>>13 & 0xFFFF,\
		((VEHICLE_ANGLE_ADDR & 0xFFFF) <<3) | 0x4,\
		0xFF<<3 | 0xF,\
		0xFF00<<3,\
		CAN_FILTER_FIFO1, 16, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,14
	};
	HAL_CAN_ConfigFilter(hcan, &VehicleAngleCANFilter);*/
  //#endif

	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	return 0;
}

uint8_t CAN1_Init(CAN_HandleTypeDef * hcan)				//启动CAN2时必须启动CAN1，目前CAN1没有使用
{
	HAL_CAN_Start(hcan);

	return 0;
}
