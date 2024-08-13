#ifndef _stm32_flash_H
#define _stm32_flash_H

#include "system.h"

#include <stdint.h>

//�����Լ�����Ҫ����
#define STM32_FLASH_SIZE 512 	 //��ѡSTM32��FLASH������С(��λΪK)
#define MAX_FAULTS 5
//FLASH��ʼ��ַ
#define FLASH_USER_START_ADDR   0x08000000  // Flash�洢����ʼ��ַ���������MCUѡ����ʵĵ�ַ��

typedef struct {
    uint16_t faultCode;
    uint32_t timestamp; // ���ڼ�¼���Ϸ�����ʱ��
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


//#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ
// 
//vu16 STM32_FLASH_ReadHalfWord(u32 faddr); 
//void STM32_FLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
//void STM32_FLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����
					   


#endif
