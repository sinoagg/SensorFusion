#include "InternalFlash.h"

uint32_t GetFlashData_SingleUint32(uint32_t flash_addr)
{
	return FlashRead32bit(flash_addr);
}

void GetFlashData_U32(uint32_t* pdata, uint32_t flash_addr, uint8_t length)
{
	uint8_t i;
	for(i=0;i<length;i++)
	{
		*pdata=FlashRead32bit(flash_addr);
		flash_addr+=4;
	}
}

uint32_t FlashRead32bit(uint32_t ReadAddr)
{
	uint32_t rtn_value;
	rtn_value=*(__IO uint32_t*)ReadAddr;//读取4个字节.		
	return rtn_value;
}

static uint8_t FlashErase(uint32_t flash_sector_addr, uint8_t nb_sectors)
{
	uint32_t SectorError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = flash_sector_addr;
  EraseInitStruct.NbSectors = nb_sectors;
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  { 
    return 1;
  }
	__HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
	
	return 0;
}

uint8_t FlashWrite_Uint32(uint32_t flash_sector_addr, uint8_t nb_sectors, uint32_t *pData, uint32_t number)
{
	uint32_t i=0;
	uint32_t Address=flash_sector_addr;
	
	HAL_FLASH_Unlock();													//解锁
	if(FlashErase(flash_sector_addr, nb_sectors)!=0)	//擦除
		return 1;
	for(i=0;i<number;i++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *(pData+i)) == HAL_OK)
    {
      Address = Address + 4;
    }
    else
			return 1;
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock(); 
	return 0;
}
