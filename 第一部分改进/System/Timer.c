#include "stm32f10x.h"                  // Device header

void Timer_Init(void)
{
    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /*配置时钟源*/
    TIM_InternalClockConfig(TIM1);
    
    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;    // 10ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;    // 72MHz/72=1MHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    /*清除更新标志*/
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    
    /*使能更新中断*/
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    
    /*NVIC配置 - 使用最简单的配置*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 简单优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    /*TIM使能*/
    TIM_Cmd(TIM1, ENABLE);
}