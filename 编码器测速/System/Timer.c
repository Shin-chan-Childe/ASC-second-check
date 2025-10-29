#include "stm32f10x.h"                  // Device header

//extern uint16_t Num;

void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//TIM2是APB1总线的外设
	
	//选择时基单元的时钟
	TIM_InternalClockConfig(TIM2);//默认内部时钟
	
	//配置时基单元
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	//对72M进行7200的分频，得到的就是10k的计数频率，在10k的频率下，计10000个数，那不就是1s的时间嘛
	TIM_TimeBaseInitStructure.TIM_Period=10000-1;//ARR自动重装器的值,0~2^16-1
	TIM_TimeBaseInitStructure.TIM_Prescaler=7200-1;//PSC预分频器的值,0~2^16-1
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复计数器的值，高级定时器才有
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);//计时器初始会自带一次中断，要先手动把更新中断标志位清除一下
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//使能中断，开启更新中断到NVIC的通路
	
	//NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC优先级分组
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
	//启动定时器
	TIM_Cmd(TIM2,ENABLE);
}

/*
//中断函数
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}
*/
