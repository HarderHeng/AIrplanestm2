#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
//使用USART1外设，对应板子上的PB6（TX）和PB7（RX）
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

#define USART1_REC_LEN 100


void Serial_Init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //初始化USART1的时钟
	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //初始化GPIOB的时钟
	GPIO_InitTypeDef GPIO_InitInstructure;
	GPIO_InitInstructure.GPIO_Mode = GPIO_Mode_AF; //GPIO复用模式才能被USART外设所调用
	GPIO_InitInstructure.GPIO_OType = GPIO_OType_PP; //USART要用推挽
	GPIO_InitInstructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6; //初始化两个口，现在两个口同时具有输入和输出的能力，实际上只需要TX输出，RX输入 
	GPIO_InitInstructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //上拉输入
	GPIO_InitInstructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_PinAFConfig(GPIOB,6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,7,GPIO_AF_USART1);
	GPIO_Init(GPIOB, &GPIO_InitInstructure);
	
	USART_InitTypeDef USART_InitInstructure; //初始化USART外设（类似于初始化GPIO端口）
	USART_InitInstructure.USART_BaudRate = 9600;//波特率
	USART_InitInstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制
	USART_InitInstructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //USART模式，既接受也发送
	USART_InitInstructure.USART_Parity = USART_Parity_No; //校验位
	USART_InitInstructure.USART_StopBits = USART_StopBits_1; //停止位一位
	USART_InitInstructure.USART_WordLength = USART_WordLength_8b; //每一句长8bit
	USART_Init(USART1, &USART_InitInstructure);
	
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启RXNE标志位的中断,当接收寄存器不是空的时候触发中断
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitInstructure;
	NVIC_InitInstructure.NVIC_IRQChannel = USART1_IRQn; //NVIC中断通道USART1
	NVIC_InitInstructure.NVIC_IRQChannelCmd = ENABLE; //开启NVIC中断
	NVIC_InitInstructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
	NVIC_InitInstructure.NVIC_IRQChannelSubPriority = 1; //响应优先级
	NVIC_Init(&NVIC_InitInstructure);
	
	
	USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t byte) {
	USART_SendData(USART1, byte);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}


uint8_t Serial_ReceiveByte(void){
	Serial_RxFlag = 0;
	return Serial_RxData;
}
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}
uint8_t GetRxFlag(void) {
	if(Serial_RxFlag==1){
		Serial_RxFlag=0;
		return 1;
	}
	else{
		Serial_RxFlag=1;
	}
	return 0;
}
void USART_IRQHandler(void) {
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET) {
		Serial_RxData=USART_ReceiveData(USART1);
		Serial_RxFlag=1;
	}
		
}

void USART1_printf(char *fmt, ...)
{
	char buffer[USART1_REC_LEN + 1]; // 数据长度
	u8 i = 0;
	va_list arg_ptr;
	va_start(arg_ptr, fmt);
	vsnprintf(buffer, USART1_REC_LEN + 1, fmt, arg_ptr);
	// USART_SendData(USART1, (u8)buffer[0]);
	while ((i < USART1_REC_LEN) && (i < strlen(buffer)))
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
			;
		USART_SendData(USART1, (u8)buffer[i++]);
		// USART_SendData(USART1, (u8)i);
		// i++;
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			;
	}
	va_end(arg_ptr);
}

void float_to_string(float number, char *str, int decimal_places)
{
	// 边界检查
	if (decimal_places < 0)
		decimal_places = 0;

	// 处理负数
	if (number < 0)
	{
		*str++ = '-';
		number = -number;
	}

	// 提取整数部分
	int integer_part = (int)number;

	// 提取小数部分
	float fractional_part = number - (float)integer_part;

	// 将整数部分转换为字符串
	sprintf(str, "%d", integer_part);
	while (*str) // 移动到字符串末尾
		str++;

	// 如果需要小数部分，添加小数点
	if (decimal_places > 0)
	{
		*str++ = '.';

		// 按指定精度处理小数部分
		for (int i = 0; i < decimal_places; i++)
		{
			fractional_part *= 10;
			int digit = (int)fractional_part;
			*str++ = '0' + digit;
			fractional_part -= digit;
		}
	}

	// 添加字符串结束符
	*str = '\0';
}