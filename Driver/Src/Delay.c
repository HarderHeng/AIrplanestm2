#include "stm32f4xx.h" // Device header
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

void DelayInit(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // APB1的时钟

	TIM_InternalClockConfig(TIM2);									// 开启TIM3的内部时钟
	TIM_TimeBaseInitTypeDef TIM_InitBaseInitStructure;				// 时基单元初始化结构体
	TIM_InitBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 滤波器采样分频
	TIM_InitBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数方式向上计数
	TIM_InitBaseInitStructure.TIM_Period = 50000 - 1;				// 自动重装器的值
	TIM_InitBaseInitStructure.TIM_Prescaler = 84 - 1;				// 分频系数:原先是84，测出来相差三倍
	TIM_InitBaseInitStructure.TIM_RepetitionCounter = 0;			// 输出分频
	TIM_TimeBaseInit(TIM2, &TIM_InitBaseInitStructure);
}
void Delay_us(uint32_t xus)
{
	TIM2->CNT = 0;
	TIM_Cmd(TIM2, ENABLE);
	while (TIM2->CNT < xus)
		;
	TIM2->CNT = 0;
	TIM_Cmd(TIM2, DISABLE);
}
void Delay_ms(uint32_t xms)
{
	while (xms--)
	{
		Delay_us(1000);
	}
}
void Delay_s(uint32_t xs)
{
	while (xs--)
	{
		Delay_ms(1000);
	}
}