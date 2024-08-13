#ifndef _can_H
#define _can_H

#include "system.h"
							    

#define CAN_RX0_INT_ENABLE 0   //��ʹ���ж�

void CAN_Mode_Init(void);//CAN��ʼ��
 
void CAN_Send_Msg(short Speed);						//��������

void CAN_Receive_Msg(void);							//��������


typedef struct { //ѹ�������в���
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

typedef struct { //ѹ�����趨����
		uint16_t speed;
		uint8_t runStatus;
		uint16_t current;
		uint8_t upSpeed;
		uint8_t downSpeed;
} Struct_CM_setMsg_InitTypedef;

typedef struct { //�߼����в���
		uint16_t lowPressure;
		uint16_t highPressure;
		uint8_t status;
		uint8_t Flag_lossPress;
		uint8_t errorCode;
		uint8_t softVersion;
		uint16_t softDate;
} Struct_logicMsg_InitTypedef;


#endif
