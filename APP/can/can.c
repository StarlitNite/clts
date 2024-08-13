#include "can.h"
#include "usart.h"
#include "stm32_flash.h"

#define Def_setStatus_Init		0x00
#define Def_setStatus_Work		0x22
#define Def_setStatus_Error		0x44

#define Def_runStatus_Init		0x11
#define Def_runStatus_Work		0x33
#define Def_runStatus_Error		0x55

#define Def_motorStatus_Init		0x00
#define Def_motorStatus_Work		0x01
#define Def_motorStatus_Error		0x02

#define Def_sysStatus_Init 	0x02
#define Def_sysStatus_Work 	0x01
#define Def_sysStatus_Stop 	0x00

extern FaultQueue faultQueue;

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+tbs2+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//������Ϊ:36M/((8+9+1)*4)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��;

Struct_CM_setMsg_InitTypedef Val_Struct_CM_Set; 
Struct_CM_runMsg_InitTypedef Val_Struct_CM_Run;
Struct_logicMsg_InitTypedef Val_Struct_Logic;

void CAN_Mode_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
#if CAN_RX0_INT_ENABLE 
	NVIC_InitTypeDef  		NVIC_InitStructure;
#endif
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //��CAN1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //PA�˿�ʱ�Ӵ�
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;		//PA11	   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	 //��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//PA12	   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//ʹ�ñ����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_8tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=4;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1
	
	//���ù�����
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
	
#if CAN_RX0_INT_ENABLE 
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
	for(i=0;i<8;i++)
	printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
}
#endif

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
int num = 0;
void CAN_Send_Msg(short Speed)
{	
	uint8_t Xor_Number,i;		
		uint8_t TxBuf[16];		//���ͻ���������
		CanTxMsg Val_CAN_TxMsg;
	
		TxBuf[0] = 0xF1;		//����֡ͷ
		TxBuf[1] = 0xAC;		//����֡ID
	
	
	  Val_Struct_CM_Set.speed=Speed;
		TxBuf[2] = (uint8_t)(Val_Struct_CM_Set.speed >> 8);		//�趨ת�ٸ�8λ
		TxBuf[3] = (uint8_t)Val_Struct_CM_Set.speed;					//�趨ת�ٵ�8λ
		TxBuf[4] = (uint8_t)(Val_Struct_CM_Set.current >> 8);		//�趨�������޸�8λ
		TxBuf[5] = (uint8_t)Val_Struct_CM_Set.current;		//�趨�������޵�8λ
		TxBuf[6] = Val_Struct_CM_Set.upSpeed;		//�趨���ٶ�
		TxBuf[7] = Val_Struct_CM_Set.downSpeed;		//�趨���ٶ�
	
		TxBuf[8] = Def_setStatus_Work;		//���ù���ģʽ��������
		TxBuf[9] = (uint8_t)(Val_Struct_Logic.lowPressure >> 8);				//��չ
		TxBuf[10] = (uint8_t)Val_Struct_Logic.lowPressure;			//��չ
		TxBuf[11] = Val_Struct_Logic.errorCode;			//��չ
		TxBuf[12] = Val_Struct_Logic.softVersion;			//��չ
		TxBuf[13] = 0;			//��չ
		TxBuf[14] = 0xFF;		//��չ
	
		for (i = 0; i < 15; i++) {
		
				TxBuf[15] ^= TxBuf[i];
		}
		
		Val_CAN_TxMsg.StdId = 0xAC;								//��׼ID��
		Val_CAN_TxMsg.RTR = CAN_RTR_DATA;					//����ģʽ
		Val_CAN_TxMsg.IDE = CAN_ID_STD;						//�Ƿ���չ֡
		Val_CAN_TxMsg.ExtId = 0xff;								//��չID��
		Val_CAN_TxMsg.DLC = 8;										//���ݳ���		
		Val_CAN_TxMsg.Data[0] = TxBuf[0];					//���͵�����
		Val_CAN_TxMsg.Data[1] = TxBuf[1];
		Val_CAN_TxMsg.Data[2] = TxBuf[2];
		Val_CAN_TxMsg.Data[3] = TxBuf[3];
		Val_CAN_TxMsg.Data[4] = TxBuf[4];
		Val_CAN_TxMsg.Data[5] = TxBuf[5];
		Val_CAN_TxMsg.Data[6] = TxBuf[6];
		Val_CAN_TxMsg.Data[7] = TxBuf[7];

		CAN_ClearFlag(CAN1,CAN_FLAG_LEC);
		
		Val_CAN_TxMsg.StdId = 0xAC;								//��׼ID��
		Val_CAN_TxMsg.RTR = CAN_RTR_DATA;					//����ģʽ
		Val_CAN_TxMsg.IDE = CAN_ID_STD;						//�Ƿ���չ֡
		Val_CAN_TxMsg.ExtId = 0xff;								//��չID��
		Val_CAN_TxMsg.DLC = 8;										//���ݳ���		
		Val_CAN_TxMsg.Data[0] = TxBuf[0];					//���͵�����
		Val_CAN_TxMsg.Data[1] = TxBuf[1];
		Val_CAN_TxMsg.Data[2] = TxBuf[2];
		Val_CAN_TxMsg.Data[3] = TxBuf[3];
		Val_CAN_TxMsg.Data[4] = TxBuf[4];
		Val_CAN_TxMsg.Data[5] = TxBuf[5];
		Val_CAN_TxMsg.Data[6] = TxBuf[6];
		Val_CAN_TxMsg.Data[7] = TxBuf[7];		
		
		CAN_Transmit(CAN1,&Val_CAN_TxMsg);
		
		Val_CAN_TxMsg.StdId = 0xAC;								//��׼ID��
		Val_CAN_TxMsg.RTR = CAN_RTR_DATA;					//����ģʽ
		Val_CAN_TxMsg.IDE = CAN_ID_STD;						//�Ƿ���չ֡
		Val_CAN_TxMsg.ExtId = 0xff;								//��չID��
		Val_CAN_TxMsg.DLC = 8;										//���ݳ���
		
		Val_CAN_TxMsg.Data[0] = TxBuf[8];					//���͵�����
		Val_CAN_TxMsg.Data[1] = TxBuf[9];
		Val_CAN_TxMsg.Data[2] = TxBuf[10];
		Val_CAN_TxMsg.Data[3] = TxBuf[11];
		Val_CAN_TxMsg.Data[4] = TxBuf[12];
		Val_CAN_TxMsg.Data[5] = TxBuf[13];
		Val_CAN_TxMsg.Data[6] = TxBuf[14];
		Val_CAN_TxMsg.Data[7] = TxBuf[15];	
		
		CAN_Transmit(CAN1,&Val_CAN_TxMsg);
		
}

//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
CanRxMsg Val_Struct_CAN_RxMsg;
void CAN_Receive_Msg()
{		   		   
     uint8_t Data_ID = 0x00;				//�ж��Ƿ����CAN���ݣ�0xFFΪ��ʼ״̬�������һ֡���գ���Ϊ0xAX���ڶ�֡���գ�Ϊ0xXA��

		if(CAN_MessagePending(CAN1, CAN_FIFO0) > 1) {
			
//            memset(&Val_Struct_CAN_RxMsg, 0, sizeof(Val_Struct_CAN_RxMsg));

				CAN_Receive(CAN1, CAN_FIFO0, &Val_Struct_CAN_RxMsg);	//����CAN2���ݼĴ�����ֵ������������Ϣ������

				if((Val_Struct_CAN_RxMsg.Data[0] == 0xF1) && (Val_Struct_CAN_RxMsg.Data[1] == 0xCC)) { //�ж�����֡�ǵ�һ֡����ȡ��Ӧ������(Val_Struct_CAN_RxMsg.Data[0] == 0xF1) && (Val_Struct_CAN_RxMsg.Data[1] == 0xCC)
											
						Val_Struct_CM_Run.speed = ((uint16_t)Val_Struct_CAN_RxMsg.Data[2] << 8) + Val_Struct_CAN_RxMsg.Data[3];//
						Val_Struct_CM_Run.current = ((uint16_t)Val_Struct_CAN_RxMsg.Data[4] << 8) + Val_Struct_CAN_RxMsg.Data[5];
						Val_Struct_CM_Run.volt = Val_Struct_CAN_RxMsg.Data[7];
						Val_Struct_CM_Run.softVersion = Val_Struct_CAN_RxMsg.Data[6];

						Data_ID |= 0xA0;
					
					//return Val_Struct_CM_Run.speed;
				}
				else {
				
				}
				if((Val_Struct_CAN_RxMsg.Data[0] == 0x00) && (Val_Struct_CAN_RxMsg.Data[6] == 0xFF)) { //�ж�����֡�ǵڶ�֡����ȡ��Ӧ������  
			
						Val_Struct_CM_Run.errorCode = Val_Struct_CAN_RxMsg.Data[1];
						Val_Struct_CM_Run.motorStatus = Val_Struct_CAN_RxMsg.Data[2];
						Val_Struct_CM_Run.runStatus = Val_Struct_CAN_RxMsg.Data[3];
						Val_Struct_CM_Run.softDate = ((uint16_t)Val_Struct_CAN_RxMsg.Data[4] << 8) + Val_Struct_CAN_RxMsg.Data[5];

						Data_ID |= 0x0A;

						if(Val_Struct_CM_Run.errorCode!= 0x00&& Val_Struct_CM_Run.errorCode!=0xff){
							//���ϴ���д���ڴ�
							//addFault(&faultQueue,01,0);
						}
				}
				else {
			
				}


		}
		else {

		}
		//return 0;
}


