#ifndef _stm32_flash_H
#define _stm32_flash_H

#include "system.h"

#include <stdint.h>

//根据自己的需要设置
#define STM32_FLASH_SIZE 512 	 //所选STM32的FLASH容量大小(单位为K)
#define MAX_FAULTS 5
//FLASH起始地址
#define FLASH_USER_START_ADDR   0x08000000  // Flash存储器起始地址（根据你的MCU选择合适的地址）

typedef struct {
    uint16_t faultCode;
    uint32_t timestamp; // 用于记录故障发生的时间
} Fault;

typedef struct {
    Fault faults[MAX_FAULTS];
    uint8_t head;
    uint8_t count;
} FaultQueue;

void initFaultQueue(FaultQueue *queue);
void addFault(FaultQueue *queue, uint16_t faultCode, uint16_t timestamp);
void loadFaultsFromFlash(FaultQueue *queue);
void saveFaultsToFlash(FaultQueue *queue);
void displayFaults(FaultQueue *queue);


//#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
// 
//vu16 STM32_FLASH_ReadHalfWord(u32 faddr); 
//void STM32_FLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//从指定地址开始写入指定长度的数据
//void STM32_FLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//从指定地址开始读出指定长度的数据
					   


#endif
