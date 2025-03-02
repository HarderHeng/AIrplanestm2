#include "stm32f4xx.h" // Device header
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "Timer.h"
#include "stm32f4xx_it.h"
#include "misc.h"
#include "pid.h"
#include "USART.h"

int PWM_IN_CH[4]; // 一共有四个通道

// 定时器1通道1输入捕获配置
/**
 * 小端
 * 右手左右：通道一占空比在801--960---1118
 * 左手上下：通道二占空比在799---958---1118
 * 右手上下：通道三油门：在640-958--1277
 * 左手左右：通道四占空比在798-958--1119
 * 大端：
 * 右手左右：通道一占空比在1000--1504---1994  	【0】
 * 左手上下：通道二占空比在997---1495---1993		【1】
 * 右手上下：通道三油门：在997-1497--1989		【2】
 * 左手左右：通道四占空比在997-1490--1993		【3】
 *
 * 以960为轴，上下320°的偏差
 * 接收机通道3是油门通道，占空比会影响四个电机，同时增加或减少转速
 * 接收机通道1是航向通道 yaw
 * 接收机通道2是俯仰通道，绕y轴，pitch 正方向M12+，M34-
 * 接收机通道4是横滚通道，绕x轴，roll正方向M14+，M23-
 */

uint8_t OC1Get_Duty(void) // 返回一个640-1280之间的数字
{						  // 获取通道1占空比
	return PWM_IN_CH[0];
}
uint8_t OC2Get_Duty(void) // 返回一个640-1280之间的数字
{						  // 获取通道2占空比
	return PWM_IN_CH[1];
}
uint8_t OC3Get_Duty(void) // 返回一个640-1280之间的数字
{						  // 获取通道3占空比
	return PWM_IN_CH[2];
}
uint8_t OC4Get_Duty(void) // 返回一个640-1280之间的数字
{						  // 获取通道4占空比
	return PWM_IN_CH[3];
}

void Receiver_display()
{

	char ch1[10];
	char ch2[10];
	char ch3[10];
	char ch4[10];

	float_to_string((float)PWM_IN_CH[0], ch1, 1);
	float_to_string((float)PWM_IN_CH[1], ch2, 1);
	float_to_string((float)PWM_IN_CH[2], ch3, 1);
	float_to_string((float)PWM_IN_CH[3], ch4, 1);
	USART1_printf("ch1:%s ", ch1);
	USART1_printf("ch2:%s ", ch2);
	USART1_printf("ch3:%s ", ch3);
	USART1_printf("ch4:%s \n", ch4);
}

/**
 * @brief 转换占空比为-1~0~1的一个值，单位化占空比
 */
void duty_to_1(float *target)
{
	if (*target < 997)
	{
		*target = 997;
	}
	else if (*target > 1990)
	{
		*target = 1990;
	}
	*target = (*target - 1495) / 496.0;
}

/**
 * @brief 转换单位占空比为角度并输出
 */
void Receiver_displaytarget()
{
	char ch1[10];
	char ch2[10];
	char ch4[10];
	char ch3[10];

	float x, y, z, t;
	Target_Read_Receiver(&x, &y, &z, &t);

	float_to_string(z, ch1, 1);
	float_to_string(y, ch2, 1);
	float_to_string(x, ch4, 1);
	float_to_string(t, ch3, 1);
	USART1_printf("ch1:Z:%s ", ch1);
	USART1_printf("ch2:Y:%s ", ch2);
	USART1_printf("ch4:X:%s ", ch4);
	USART1_printf("thro:%s\n", ch3);
}

/**
 * @brief 外部获取receiver的期待x、y、z,得到一个单位化的占空比
 * @param target_x +-30
 * @param target_y +-30
 * @param target_z +-30
 * @param thro 0-1
 */
void Target_Read_Receiver(float *target_x, float *target_y, float *target_z, float *Thro)
{
	float tempx, tempy, tempz, tempT;
	// tempx = (float)OC4Get_Duty();
	// tempy = (float)OC2Get_Duty();
	// tempz = (float)OC1Get_Duty();
	// tempT = (float)OC3Get_Duty(); // 获得油门参数
	tempx = PWM_IN_CH[3];
	tempy = PWM_IN_CH[1];
	tempz = PWM_IN_CH[0];
	tempT = PWM_IN_CH[2];
	duty_to_1(&tempx);
	duty_to_1(&tempy);
	duty_to_1(&tempz);
	tempT = (tempT - 997) / 990;
	// duty_to_1(&tempT); // 单位化

	*target_x = tempx * 30;
	*target_y = tempy * 30;
	*target_z = tempz * 30;
	*Thro = tempT; // 赋值
}

/**
 * @brief 接收机输入捕获初始化
 */
void IC_Init(void)
{
	/**开启外设时钟**/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/**配置GPIOA**/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/**复用GPIOA为TIM1**/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

	/**初始化TIM1，开启定时器内部时钟**/
	TIM_DeInit(TIM1);

	/**TIM1时基单元配置**/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1; // ARR设为5000-1
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1; // 记一次数需要(1/1M)s
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_ICInitTypeDef TIM_ICInitStructure;

	// TIM1->CH1
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// TIM1->CH2
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// TIM1->CH3
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICFilter = 0x00; // 原先是0x0b
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// TIM1->CH4
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_Update);

	/**配置NVIC**/
	NVIC_InitTypeDef NVIC_InitTypeDefStructure;
	NVIC_InitTypeDefStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn | TIM1_CC_IRQn;
	NVIC_InitTypeDefStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitTypeDefStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitTypeDefStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitTypeDefStructure);

	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM1, DISABLE);

	/**中断使能**/
	TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_Update, ENABLE);

	// 使能定时器
	TIM_Cmd(TIM1, ENABLE);
}

uint8_t TIM1_CAPTURE_STA[4] = {0};	   // 初始值为0。0：此前为低电平,说明此时捕获到上升沿; 1: 此前为高电平，说明此时捕获下降沿
uint16_t TIM1_CAPTURE_OVF[4] = {0};	   // CHx高电平期间，计数器溢出次数
uint16_t TIM1_CAPTURE_VAL[4][2] = {0}; // CHx捕获到上升沿与下降沿的CCRx的值 val[0]表示捕获到上升沿的CCR值 val[1]表示捕获到下降沿的CCR值

void TIM1_UP_TIM10_IRQHandler(void)
{
	// 更新中断处理
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除更新中断标志位
		uint8_t i;
		for (i = 0; i < 4; i++)
		{
			if (TIM1_CAPTURE_STA[i] == 1) // 判断CHx已经处在高电平
			{
				if ((TIM1_CAPTURE_OVF[i] & 0x3F) == 0x3F) // 高电平时间过长
				{
					TIM1_CAPTURE_VAL[i][0] = 0; // 捕获时间舍弃
					TIM1_CAPTURE_OVF[i] = 0;
				}
				else
				{
					TIM1_CAPTURE_OVF[i]++; // 溢出次数加一
				}
			}
		}
	}
}

void TIM1_CC_IRQHandler(void)
{
	// CH1捕获
	if (TIM_GetFlagStatus(TIM1, TIM_IT_CC1) != RESET) // 通道一发生捕获
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); // 清除捕获中断标志位
		if (TIM1_CAPTURE_STA[0] == 0)			 // 捕获前为低电平,捕获上升沿
		{
			TIM1_CAPTURE_VAL[0][0] = TIM_GetCapture1(TIM1);		 // 存取上升沿的CNT值
			TIM1_CAPTURE_STA[0] = 1;							 // 更改捕获状态
			TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling); // 通道下降沿捕获
		}
		else
		{
			TIM1_CAPTURE_VAL[0][1] = TIM_GetCapture1(TIM1); // 存取下降沿的CNT值
			if (TIM1_CAPTURE_VAL[0][1] > TIM1_CAPTURE_VAL[0][0])
			{
				PWM_IN_CH[0] = TIM1_CAPTURE_VAL[0][1] - TIM1_CAPTURE_VAL[0][0] + TIM1_CAPTURE_OVF[0] * 4999;
			}
			else
			{
				PWM_IN_CH[0] = TIM1_CAPTURE_OVF[0] * 4999 - TIM1_CAPTURE_VAL[0][0] + TIM1_CAPTURE_VAL[0][1];
			}
			TIM1_CAPTURE_OVF[0] = 0;							// 溢出次数清零
			TIM1_CAPTURE_STA[0] = 0;							// 更改捕获状态
			TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); // 通道上升沿沿捕获
		}
	}

	// CH2捕获
	if (TIM_GetFlagStatus(TIM1, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
		if (TIM1_CAPTURE_STA[1] == 0)
		{
			TIM1_CAPTURE_VAL[1][0] = TIM_GetCapture2(TIM1);
			TIM1_CAPTURE_STA[1] = 1;
			TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling);
		}
		else
		{
			TIM1_CAPTURE_VAL[1][1] = TIM_GetCapture2(TIM1);
			if (TIM1_CAPTURE_VAL[1][1] > TIM1_CAPTURE_VAL[1][0])
			{
				PWM_IN_CH[1] = TIM1_CAPTURE_VAL[1][1] - TIM1_CAPTURE_VAL[1][0] + TIM1_CAPTURE_OVF[1] * 4999;
			}
			else
			{
				PWM_IN_CH[1] = TIM1_CAPTURE_OVF[1] * 4999 - TIM1_CAPTURE_VAL[1][0] + TIM1_CAPTURE_VAL[1][1];
			}
			TIM1_CAPTURE_OVF[1] = 0;
			TIM1_CAPTURE_STA[1] = 0;
			TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising);
		}
	}

	// CH3捕获
	if (TIM_GetFlagStatus(TIM1, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
		if (TIM1_CAPTURE_STA[2] == 0)
		{
			TIM1_CAPTURE_VAL[2][0] = TIM_GetCapture3(TIM1);
			TIM1_CAPTURE_STA[2] = 1;
			TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling);
		}
		else
		{
			TIM1_CAPTURE_VAL[2][1] = TIM_GetCapture3(TIM1);
			if (TIM1_CAPTURE_VAL[2][1] > TIM1_CAPTURE_VAL[2][0])
			{
				PWM_IN_CH[2] = TIM1_CAPTURE_VAL[2][1] - TIM1_CAPTURE_VAL[2][0] + TIM1_CAPTURE_OVF[2] * 4999;
			}
			else
			{
				PWM_IN_CH[2] = TIM1_CAPTURE_OVF[2] * 4999 - TIM1_CAPTURE_VAL[2][0] + TIM1_CAPTURE_VAL[2][1];
			}
			TIM1_CAPTURE_OVF[2] = 0;
			TIM1_CAPTURE_STA[2] = 0;
			TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising);
		}
	}

	// CH4捕获
	if (TIM_GetFlagStatus(TIM1, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
		if (TIM1_CAPTURE_STA[3] == 0)
		{
			TIM1_CAPTURE_VAL[3][0] = TIM_GetCapture4(TIM1);
			TIM1_CAPTURE_STA[3] = 1;
			TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling);
		}
		else
		{
			TIM1_CAPTURE_VAL[3][1] = TIM_GetCapture4(TIM1);
			if (TIM1_CAPTURE_VAL[3][1] > TIM1_CAPTURE_VAL[3][0])
			{
				PWM_IN_CH[3] = TIM1_CAPTURE_VAL[3][1] - TIM1_CAPTURE_VAL[3][0] + TIM1_CAPTURE_OVF[3] * 4999;
			}
			else
			{
				PWM_IN_CH[3] = TIM1_CAPTURE_OVF[3] * 4999 - TIM1_CAPTURE_VAL[3][0] + TIM1_CAPTURE_VAL[3][1];
			}
			TIM1_CAPTURE_OVF[3] = 0;
			TIM1_CAPTURE_STA[3] = 0;
			TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising);
		}
	}
}
