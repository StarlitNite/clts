#include "stm32_flash.h"
#include "stm32f10x_flash.h"
#include "tftlcd.h"
#include <stdio.h>
////��ȡָ����ַ�İ���(16λ����)
////faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
////����ֵ:��Ӧ����.
//vu16 STM32_FLASH_ReadHalfWord(u32 faddr)
//{
//	return *(vu16*)faddr; 
//} 

////������д��
////WriteAddr:��ʼ��ַ
////pBuffer:����ָ��
////NumToWrite:����(16λ)��   
//void STM32_FLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
//{ 			 		 
//	u16 i;
//	for(i=0;i<NumToWrite;i++)
//	{
//		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
//	    WriteAddr+=2;//��ַ����2.
//	}  
//}

////��ָ����ַ��ʼд��ָ�����ȵ�����
////WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���)
////pBuffer:����ָ��
////NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
//#if STM32_FLASH_SIZE<256
//	#define STM32_SECTOR_SIZE 1024 //�ֽ�
//#else 
//	#define STM32_SECTOR_SIZE	2048
//#endif		 
//u16 STM32_FLASH_BUF[STM32_SECTOR_SIZE/2];//�����2K�ֽ�
//void STM32_FLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
//{
//	u32 secpos;	   //������ַ
//	u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
//	u16 secremain; //������ʣ���ַ(16λ�ּ���)	   
// 	u16 i;    
//	u32 offaddr;   //ȥ��0X08000000��ĵ�ַ
//	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
//	FLASH_Unlock();						//����
//	offaddr=WriteAddr-STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
//	secpos=offaddr/STM32_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
//	secoff=(offaddr%STM32_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
//	secremain=STM32_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С   
//	if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ
//	while(1) 
//	{	
//		STM32_FLASH_Read(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE,STM32_FLASH_BUF,STM32_SECTOR_SIZE/2);//������������������
//		for(i=0;i<secremain;i++)//У������
//		{
//			if(STM32_FLASH_BUF[secoff+i]!=0XFFFF)
//				break;//��Ҫ����  	  
//		}
//		if(i<secremain)//��Ҫ����
//		{
//			FLASH_ErasePage(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
//			for(i=0;i<secremain;i++)//����
//			{
//				STM32_FLASH_BUF[i+secoff]=pBuffer[i];	  
//			}
//			STM32_FLASH_Write_NoCheck(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE,STM32_FLASH_BUF,STM32_SECTOR_SIZE/2);//д����������  
//		}
//		else 
//			STM32_FLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
//		if(NumToWrite==secremain)
//			break;//д�������
//		else//д��δ����
//		{
//			secpos++;				//������ַ��1
//			secoff=0;				//ƫ��λ��Ϊ0 	 
//		   	pBuffer+=secremain;  	//ָ��ƫ��
//			WriteAddr+=secremain;	//д��ַƫ��	   
//		   	NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
//			if(NumToWrite>(STM32_SECTOR_SIZE/2))
//				secremain=STM32_SECTOR_SIZE/2;//��һ����������д����
//			else 
//				secremain=NumToWrite;//��һ����������д����
//		}	 
//	}	
//	FLASH_Lock();//����
//}

////��ָ����ַ��ʼ����ָ�����ȵ�����
////ReadAddr:��ʼ��ַ
////pBuffer:����ָ��
////NumToWrite:����(16λ)��
//void STM32_FLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
//{
//	u16 i;
//	for(i=0;i<NumToRead;i++)
//	{
//		pBuffer[i]=STM32_FLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
//		ReadAddr+=2;//ƫ��2���ֽ�.	
//	}
//}
//��ʼ�����϶���
/*void initFaultQueue(FaultQueue *queue){
		queue->count=0;
		queue->head=0;
}

//����д���ڴ�
int x =0 ,y=0;
void addFault(FaultQueue *queue,uint16_t faultCode,uint16_t timestamp){
		
		uint8_t index = (queue->head + queue->count) % MAX_FAULTS;
    
    queue->faults[index].faultCode = faultCode;
    queue->faults[index].timestamp = timestamp;

    if (queue->count < MAX_FAULTS) {
        queue->count++;
    } else {
        queue->head = (queue->head + 1) % MAX_FAULTS;
    }
    x=queue->head;
		y=queue->count;
    // ��ӹ��Ϻ󱣴浽Flash
    saveFaultsToFlash(queue);
}

//���ڴ��м��ع���
void loadFaultsFromFlash(FaultQueue *queue) {
    // ��Flash��ȡ����
    uint16_t *flash_addr = (uint16_t *)FLASH_USER_START_ADDR;
    uint16_t data_size = sizeof(FaultQueue) / 2;

    uint16_t *queue_ptr = (uint16_t *)queue;
    for (uint16_t i = 0; i < data_size; i++) {
        queue_ptr[i] = flash_addr[i];
    }
}

//���ڴ���д�����
int size=0;
void saveFaultsToFlash(FaultQueue *queue) {
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    // ����Flashҳ
    FLASH_Status status = FLASH_ErasePage(FLASH_USER_START_ADDR);
    if (status != FLASH_COMPLETE) {
        // Handle the error
        FLASH_Lock();
        return;
    }
		
    // д��Flash
    uint16_t *queue_ptr = (uint16_t *)queue;
    uint16_t data_size = sizeof(FaultQueue) / 2;
		size = data_size;
    for (uint16_t i = 0; i < data_size; i++) {
        status = FLASH_ProgramHalfWord(FLASH_USER_START_ADDR + i * 2, queue_ptr[i]);
        if (status != FLASH_COMPLETE) {
            // Handle the error
            FLASH_Lock();
            return;
		}
    }
    FLASH_Lock();
}

void displayFaults(FaultQueue *queue) {
    //printf("Recent Faults:\n");
		uint8_t y=15,x=155;
    for (uint8_t i = 0; i < queue->count; i++) {//queue->count
        uint8_t index = (queue->head + i) % MAX_FAULTS;
				LCD_ShowNum(y-10,x,i+1,1,24);
        LCD_ShowNum(y,x,queue->faults[index].faultCode,4,24);//queue->faults[index].faultCode
				LCD_ShowNum(y+45,x,queue->faults[index].timestamp,8,24);//queue->faults[index].timestamp
				x+=25;
			//printf("Fault Code: %u, queue->faults[index].faultCodeTimestamp: %u\n", queue->faults[index].faultCode, queue->faults[index].timestamp);
    }
}
*/