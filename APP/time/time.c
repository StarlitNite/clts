#include "time.h"
#include "can.h"
#include "stm32f10x.h"


void TIM4_Init(u16 per, u16 psc){
	
			TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
			NVIC_InitTypeDef NVIC_InitStruct;
	
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

			TIM_TimeBaseInitStruct.TIM_Period=per;
			TIM_TimeBaseInitStruct.TIM_Prescaler=psc;
			TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
			TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
			TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
			
			TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//开启定时器中断
			TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	
	
			NVIC_InitStruct.NVIC_IRQChannel=TIM4_IRQn;
			NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;
			NVIC_InitStruct.NVIC_IRQChannelSubPriority=3;
			NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
			NVIC_Init(&NVIC_InitStruct);
	
			TIM_Cmd(TIM4,ENABLE);
}

void TIM4_IRQHandler(void){

			if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET){//start_flag==1 && 
				if(start_flag==1){
					CAN_Send_Msg(Set_Speed);
				}else{
				}
			TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
				
			}
			
}

uint32_t getCurrentTimestamp() {
    return SysTick->VAL; // 返回当前SysTick计数值（根据实际需求修改）
}