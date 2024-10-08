#include "stm32_flash.h"
#include "stm32f10x_flash.h"
#include "tftlcd.h"
#include <stdio.h>
////读取指定地址的半字(16位数据)
////faddr:读地址(此地址必须为2的倍数!!)
////返回值:对应数据.
//vu16 STM32_FLASH_ReadHalfWord(u32 faddr)
//{
//	return *(vu16*)faddr; 
//} 

////不检查的写入
////WriteAddr:起始地址
////pBuffer:数据指针
////NumToWrite:半字(16位)数   
//void STM32_FLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
//{ 			 		 
//	u16 i;
//	for(i=0;i<NumToWrite;i++)
//	{
//		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
//	    WriteAddr+=2;//地址增加2.
//	}  
//}

////从指定地址开始写入指定长度的数据
////WriteAddr:起始地址(此地址必须为2的倍数)
////pBuffer:数据指针
////NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
//#if STM32_FLASH_SIZE<256
//	#define STM32_SECTOR_SIZE 1024 //字节
//#else 
//	#define STM32_SECTOR_SIZE	2048
//#endif		 
//u16 STM32_FLASH_BUF[STM32_SECTOR_SIZE/2];//最多是2K字节
//void STM32_FLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
//{
//	u32 secpos;	   //扇区地址
//	u16 secoff;	   //扇区内偏移地址(16位字计算)
//	u16 secremain; //扇区内剩余地址(16位字计算)	   
// 	u16 i;    
//	u32 offaddr;   //去掉0X08000000后的地址
//	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
//	FLASH_Unlock();						//解锁
//	offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
//	secpos=offaddr/STM32_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
//	secoff=(offaddr%STM32_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
//	secremain=STM32_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小   
//	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
//	while(1) 
//	{	
//		STM32_FLASH_Read(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE,STM32_FLASH_BUF,STM32_SECTOR_SIZE/2);//读出整个扇区的内容
//		for(i=0;i<secremain;i++)//校验数据
//		{
//			if(STM32_FLASH_BUF[secoff+i]!=0XFFFF)
//				break;//需要擦除  	  
//		}
//		if(i<secremain)//需要擦除
//		{
//			FLASH_ErasePage(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
//			for(i=0;i<secremain;i++)//复制
//			{
//				STM32_FLASH_BUF[i+secoff]=pBuffer[i];	  
//			}
//			STM32_FLASH_Write_NoCheck(secpos*STM32_SECTOR_SIZE+STM32_FLASH_BASE,STM32_FLASH_BUF,STM32_SECTOR_SIZE/2);//写入整个扇区  
//		}
//		else 
//			STM32_FLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
//		if(NumToWrite==secremain)
//			break;//写入结束了
//		else//写入未结束
//		{
//			secpos++;				//扇区地址增1
//			secoff=0;				//偏移位置为0 	 
//		   	pBuffer+=secremain;  	//指针偏移
//			WriteAddr+=secremain;	//写地址偏移	   
//		   	NumToWrite-=secremain;	//字节(16位)数递减
//			if(NumToWrite>(STM32_SECTOR_SIZE/2))
//				secremain=STM32_SECTOR_SIZE/2;//下一个扇区还是写不完
//			else 
//				secremain=NumToWrite;//下一个扇区可以写完了
//		}	 
//	}	
//	FLASH_Lock();//上锁
//}

////从指定地址开始读出指定长度的数据
////ReadAddr:起始地址
////pBuffer:数据指针
////NumToWrite:半字(16位)数
//void STM32_FLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
//{
//	u16 i;
//	for(i=0;i<NumToRead;i++)
//	{
//		pBuffer[i]=STM32_FLASH_ReadHalfWord(ReadAddr);//读取2个字节.
//		ReadAddr+=2;//偏移2个字节.	
//	}
//}
//初始化故障队列
/*void initFaultQueue(FaultQueue *queue){
		queue->count=0;
		queue->head=0;
}

//故障写入内存
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
    // 添加故障后保存到Flash
    saveFaultsToFlash(queue);
}

//从内存中加载故障
void loadFaultsFromFlash(FaultQueue *queue) {
    // 从Flash读取数据
    uint16_t *flash_addr = (uint16_t *)FLASH_USER_START_ADDR;
    uint16_t data_size = sizeof(FaultQueue) / 2;

    uint16_t *queue_ptr = (uint16_t *)queue;
    for (uint16_t i = 0; i < data_size; i++) {
        queue_ptr[i] = flash_addr[i];
    }
}

//向内存中写入故障
int size=0;
void saveFaultsToFlash(FaultQueue *queue) {
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    // 擦除Flash页
    FLASH_Status status = FLASH_ErasePage(FLASH_USER_START_ADDR);
    if (status != FLASH_COMPLETE) {
        // Handle the error
        FLASH_Lock();
        return;
    }
		
    // 写入Flash
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