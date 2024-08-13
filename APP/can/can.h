#ifndef _can_H
#define _can_H

#include "system.h"
							    

#define CAN_RX0_INT_ENABLE 0   //不使用中断

void CAN_Mode_Init(void);//CAN初始化
 
void CAN_Send_Msg(short Speed);						//发送数据

void CAN_Receive_Msg(void);							//接收数据


typedef struct { //压缩机运行参数
		uint16_t speed;
		uint8_t runStatus;
		uint16_t current;
		uint8_t volt;
		uint8_t motorStatus;
		uint8_t errorCode;
		uint8_t softVersion;
		uint16_t softDate;
} Struct_CM_runMsg_InitTypedef;

extern Struct_CM_runMsg_InitTypedef Val_Struct_CM_Run;

typedef struct { //压缩机设定参数
		uint16_t speed;
		uint8_t runStatus;
		uint16_t current;
		uint8_t upSpeed;
		uint8_t downSpeed;
} Struct_CM_setMsg_InitTypedef;

typedef struct { //逻辑运行参数
		uint16_t lowPressure;
		uint16_t highPressure;
		uint8_t status;
		uint8_t Flag_lossPress;
		uint8_t errorCode;
		uint8_t softVersion;
		uint16_t softDate;
} Struct_logicMsg_InitTypedef;


#endif
