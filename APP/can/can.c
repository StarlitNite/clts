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

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;  tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+tbs2+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//则波特率为:36M/((8+9+1)*4)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败;

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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //打开CAN1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //PA端口时钟打开
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;		//PA11	   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	 //上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//PA12	   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//使用报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_8tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=4;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1
	
	//配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
	
#if CAN_RX0_INT_ENABLE 
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0消息挂号中断允许.		    

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

#if CAN_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
	for(i=0;i<8;i++)
	printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
int num = 0;
void CAN_Send_Msg(short Speed)
{	
	uint8_t Xor_Number,i;		
		uint8_t TxBuf[16];		//发送缓冲区数组
		CanTxMsg Val_CAN_TxMsg;
	
		TxBuf[0] = 0xF1;		//数据帧头
		TxBuf[1] = 0xAC;		//数据帧ID
	
	
	  Val_Struct_CM_Set.speed=Speed;
		TxBuf[2] = (uint8_t)(Val_Struct_CM_Set.speed >> 8);		//设定转速高8位
		TxBuf[3] = (uint8_t)Val_Struct_CM_Set.speed;					//设定转速低8位
		TxBuf[4] = (uint8_t)(Val_Struct_CM_Set.current >> 8);		//设定电流门限高8位
		TxBuf[5] = (uint8_t)Val_Struct_CM_Set.current;		//设定电流门限低8位
		TxBuf[6] = Val_Struct_CM_Set.upSpeed;		//设定加速度
		TxBuf[7] = Val_Struct_CM_Set.downSpeed;		//设定减速度
	
		TxBuf[8] = Def_setStatus_Work;		//设置工作模式————
		TxBuf[9] = (uint8_t)(Val_Struct_Logic.lowPressure >> 8);				//扩展
		TxBuf[10] = (uint8_t)Val_Struct_Logic.lowPressure;			//扩展
		TxBuf[11] = Val_Struct_Logic.errorCode;			//扩展
		TxBuf[12] = Val_Struct_Logic.softVersion;			//扩展
		TxBuf[13] = 0;			//扩展
		TxBuf[14] = 0xFF;		//扩展
	
		for (i = 0; i < 15; i++) {
		
				TxBuf[15] ^= TxBuf[i];
		}
		
		Val_CAN_TxMsg.StdId = 0xAC;								//标准ID号
		Val_CAN_TxMsg.RTR = CAN_RTR_DATA;					//发送模式
		Val_CAN_TxMsg.IDE = CAN_ID_STD;						//是否扩展帧
		Val_CAN_TxMsg.ExtId = 0xff;								//扩展ID号
		Val_CAN_TxMsg.DLC = 8;										//数据长度		
		Val_CAN_TxMsg.Data[0] = TxBuf[0];					//发送的数据
		Val_CAN_TxMsg.Data[1] = TxBuf[1];
		Val_CAN_TxMsg.Data[2] = TxBuf[2];
		Val_CAN_TxMsg.Data[3] = TxBuf[3];
		Val_CAN_TxMsg.Data[4] = TxBuf[4];
		Val_CAN_TxMsg.Data[5] = TxBuf[5];
		Val_CAN_TxMsg.Data[6] = TxBuf[6];
		Val_CAN_TxMsg.Data[7] = TxBuf[7];

		CAN_ClearFlag(CAN1,CAN_FLAG_LEC);
		
		Val_CAN_TxMsg.StdId = 0xAC;								//标准ID号
		Val_CAN_TxMsg.RTR = CAN_RTR_DATA;					//发送模式
		Val_CAN_TxMsg.IDE = CAN_ID_STD;						//是否扩展帧
		Val_CAN_TxMsg.ExtId = 0xff;								//扩展ID号
		Val_CAN_TxMsg.DLC = 8;										//数据长度		
		Val_CAN_TxMsg.Data[0] = TxBuf[0];					//发送的数据
		Val_CAN_TxMsg.Data[1] = TxBuf[1];
		Val_CAN_TxMsg.Data[2] = TxBuf[2];
		Val_CAN_TxMsg.Data[3] = TxBuf[3];
		Val_CAN_TxMsg.Data[4] = TxBuf[4];
		Val_CAN_TxMsg.Data[5] = TxBuf[5];
		Val_CAN_TxMsg.Data[6] = TxBuf[6];
		Val_CAN_TxMsg.Data[7] = TxBuf[7];		
		
		CAN_Transmit(CAN1,&Val_CAN_TxMsg);
		
		Val_CAN_TxMsg.StdId = 0xAC;								//标准ID号
		Val_CAN_TxMsg.RTR = CAN_RTR_DATA;					//发送模式
		Val_CAN_TxMsg.IDE = CAN_ID_STD;						//是否扩展帧
		Val_CAN_TxMsg.ExtId = 0xff;								//扩展ID号
		Val_CAN_TxMsg.DLC = 8;										//数据长度
		
		Val_CAN_TxMsg.Data[0] = TxBuf[8];					//发送的数据
		Val_CAN_TxMsg.Data[1] = TxBuf[9];
		Val_CAN_TxMsg.Data[2] = TxBuf[10];
		Val_CAN_TxMsg.Data[3] = TxBuf[11];
		Val_CAN_TxMsg.Data[4] = TxBuf[12];
		Val_CAN_TxMsg.Data[5] = TxBuf[13];
		Val_CAN_TxMsg.Data[6] = TxBuf[14];
		Val_CAN_TxMsg.Data[7] = TxBuf[15];	
		
		CAN_Transmit(CAN1,&Val_CAN_TxMsg);
		
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
CanRxMsg Val_Struct_CAN_RxMsg;
void CAN_Receive_Msg()
{		   		   
     uint8_t Data_ID = 0x00;				//判断是否读到CAN数据，0xFF为初始状态，如果第一帧接收，则为0xAX，第二帧接收，为0xXA；

		if(CAN_MessagePending(CAN1, CAN_FIFO0) > 1) {
			
//            memset(&Val_Struct_CAN_RxMsg, 0, sizeof(Val_Struct_CAN_RxMsg));

				CAN_Receive(CAN1, CAN_FIFO0, &Val_Struct_CAN_RxMsg);	//读出CAN2数据寄存器的值，存入正常信息缓冲区

				if((Val_Struct_CAN_RxMsg.Data[0] == 0xF1) && (Val_Struct_CAN_RxMsg.Data[1] == 0xCC)) { //判断数据帧是第一帧，读取对应的数据(Val_Struct_CAN_RxMsg.Data[0] == 0xF1) && (Val_Struct_CAN_RxMsg.Data[1] == 0xCC)
											
						Val_Struct_CM_Run.speed = ((uint16_t)Val_Struct_CAN_RxMsg.Data[2] << 8) + Val_Struct_CAN_RxMsg.Data[3];//
						Val_Struct_CM_Run.current = ((uint16_t)Val_Struct_CAN_RxMsg.Data[4] << 8) + Val_Struct_CAN_RxMsg.Data[5];
						Val_Struct_CM_Run.volt = Val_Struct_CAN_RxMsg.Data[7];
						Val_Struct_CM_Run.softVersion = Val_Struct_CAN_RxMsg.Data[6];

						Data_ID |= 0xA0;
					
					//return Val_Struct_CM_Run.speed;
				}
				else {
				
				}
				if((Val_Struct_CAN_RxMsg.Data[0] == 0x00) && (Val_Struct_CAN_RxMsg.Data[6] == 0xFF)) { //判断数据帧是第二帧，读取对应的数据  
			
						Val_Struct_CM_Run.errorCode = Val_Struct_CAN_RxMsg.Data[1];
						Val_Struct_CM_Run.motorStatus = Val_Struct_CAN_RxMsg.Data[2];
						Val_Struct_CM_Run.runStatus = Val_Struct_CAN_RxMsg.Data[3];
						Val_Struct_CM_Run.softDate = ((uint16_t)Val_Struct_CAN_RxMsg.Data[4] << 8) + Val_Struct_CAN_RxMsg.Data[5];

						Data_ID |= 0x0A;

						if(Val_Struct_CM_Run.errorCode!= 0x00&& Val_Struct_CM_Run.errorCode!=0xff){
							//故障代码写入内存
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


