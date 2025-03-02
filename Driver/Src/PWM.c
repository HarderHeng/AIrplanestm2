#include "stm32f4xx.h" // Device header
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "Delay.h"
#include "Timer.h"
#include "USART.h"

/**
 * @brief PWM初始化
 *
 */
void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 开启APB1时钟

	TIM_InternalClockConfig(TIM3);									// 开启TIM3的内部时钟
	TIM_TimeBaseInitTypeDef TIM_InitBaseInitStructure;				// 时基单元初始化结构体
	TIM_InitBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 滤波器采样分频
	TIM_InitBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数方式向上计数
	TIM_InitBaseInitStructure.TIM_Period = 5000 - 1;				// 自动重装器的值
	TIM_InitBaseInitStructure.TIM_Prescaler = 84 - 1;				// 分频系数
	TIM_InitBaseInitStructure.TIM_RepetitionCounter = 0;			// 输出分频
	TIM_TimeBaseInit(TIM3, &TIM_InitBaseInitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;											 // GPIO初始化结构体
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									 // 复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									 // 推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;								 // 没有上拉下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;								 // 超高速
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);

	TIM_OCInitTypeDef TIM_OCInitStructure;						  // 创建OC输出结构体
	TIM_OCStructInit(&TIM_OCInitStructure);						  // 结构体自动赋初始值的函数
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // PWM模式1
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;	  // REF的极性翻转，高电平就是不翻转
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出状态开启
	TIM_OCInitStructure.TIM_Pulse = 500;						  // 指定CCR的值
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);					  // 从OC1通道输出PWM信号
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);					  // 从OC2通道输出PWM信号
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);					  // 从OC3通道输出PWM信号
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);					  // 从OC4通道输出PWM信号

	// TIM3->CCR1 = 500;
	// TIM3->CCR2 = 500;
	// TIM3->CCR3 = 500;
	// TIM3->CCR4 = 500; // 暂时还不知道这玩意到底是有用没用
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_Cmd(TIM3, ENABLE); // 开启TIM
}

/**
 * @brief PWM输出比较函数
 * @param compara 输出的高电平占空比，范围为0~500
 *
 */
void PWM_SetCompare1(uint16_t compare)
{

	TIM_SetCompare1(TIM3, compare);
}
void PWM_SetCompare2(uint16_t compare)
{

	TIM_SetCompare2(TIM3, compare);
}
void PWM_SetCompare3(uint16_t compare)
{

	TIM_SetCompare3(TIM3, compare);
}
void PWM_SetCompare4(uint16_t compare)
{

	TIM_SetCompare4(TIM3, compare);
}
/**
 * @brief 接受遥控器信号并进行信号的输出。
 *
 */
void PWM_Start(void)
{
	PWM_SetCompare1(1000);
	PWM_SetCompare2(1000);
	PWM_SetCompare3(1000);
	PWM_SetCompare4(1000);
	// Delay_ms(8000);
	// USART1_printf("pwm start");
}

void PWM_fromReceiver()
{
	int M;
	M = OC3Get_Duty();
	while (1)
	{
		PWM_SetCompare1(M);
		PWM_SetCompare2(M);
		PWM_SetCompare3(M);
		PWM_SetCompare4(M);
	}
}

void PWM_Set(int M1, int M2, int M3, int M4)
{
	// PWM_SetCompare1(M1);
	// PWM_SetCompare2(M2);
	// PWM_SetCompare3(M3);
	// PWM_SetCompare4(M4);
	TIM3->CCR1 = M1;
	TIM3->CCR2 = M2;
	TIM3->CCR3 = M3;
	TIM3->CCR4 = M4;
	// Delay_ms(6000);
	// USART1_printf("begin");
}