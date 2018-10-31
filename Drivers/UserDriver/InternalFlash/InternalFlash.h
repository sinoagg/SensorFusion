#ifndef __INTERNALFLASH_H
#define __INTERNALFLASH_H

#include "stm32f4xx_hal.h"

#define FLASH_USER_START_ADDR   FLASH_SECTOR_11   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     FLASH_SECTOR_11  +  GetSectorSize(FLASH_SECTOR_11) -1

#define RADAR_LIMIT_OFFSET_ADDR 0x00   //À×´ï¼ì²â¾àÀëÆ«ÒÆÁ¿´æ´¢µØÖ·

void GetFlashData_U32(uint32_t* pdata, uint32_t flash_addr, uint8_t length);
uint32_t GetFlashData_SingleUint32(uint32_t flash_addr);
uint32_t FlashRead32bit(uint32_t ReadAddr);
static uint8_t FlashErase(uint32_t flash_sector_addr, uint8_t nb_sectors);
uint8_t FlashWrite_Uint32(uint32_t flash_sector_addr, uint8_t nb_sectors, uint32_t *pData, uint32_t number);

#endif

