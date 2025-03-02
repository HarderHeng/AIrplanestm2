#include "stm32f4xx.h" // Device header
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "stm32f4xx_tim.h"
#include "USART.h"

/**********为了匿名四轴上位机的协议定义的变量****************************/
// cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp) (*(char *)(&dwTemp))       // 取出int型变量的低字节
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1)) //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

// #define BYTE0(dwTemp) (dwTemp & 0xFF)
// #define BYTE1(dwTemp) (dwTemp >> 8)

uint8_t BUFF[100];//发送数据帧的缓冲区大小

/*发送六个轴的数据*/
void anotc_senddata(uint16_t A, uint16_t B, uint16_t C, uint16_t D, uint16_t E, uint16_t F)
{
    int i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt = 0;
    BUFF[_cnt++] = 0xAA;     // 帧头
    BUFF[_cnt++] = 0xFF;     // 目标地址
    BUFF[_cnt++] = 0X01;     // 功能码
    BUFF[_cnt++] = 0x0D;     // 数据长度
    BUFF[_cnt++] = BYTE0(A); // 数据内容,小段模式，低位在前
    BUFF[_cnt++] = BYTE1(A); // 需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++] = BYTE0(B);
    BUFF[_cnt++] = BYTE1(B);
    BUFF[_cnt++] = BYTE0(C);
    BUFF[_cnt++] = BYTE1(C);
    BUFF[_cnt++] = BYTE0(D);
    BUFF[_cnt++] = BYTE1(D);
    BUFF[_cnt++] = BYTE0(E);
    BUFF[_cnt++] = BYTE1(E);
    BUFF[_cnt++] = BYTE0(F);
    BUFF[_cnt++] = BYTE1(F);
    // BUFF[_cnt++] = fu;
    BUFF[_cnt++] = 1;

    // SC和AC的校验直接抄最上面上面简介的即可
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    // for(i=0;i<_cnt;i++) UsartSendByte(USART2,BUFF[i]);//串口逐个发送数据
    for (i = 0; i < _cnt; i++)
    {
        // USART_SendData(USART1, BUFF[i]);
        Serial_SendByte(BUFF[i]); // usart1
    }
}

/*发送三个角度数据*/
void anotc_sendangle(uint16_t A, uint16_t B, uint16_t C)
{
    int i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt = 0;
    BUFF[_cnt++] = 0xAA;     // 帧头
    BUFF[_cnt++] = 0xFF;     // 目标地址
    BUFF[_cnt++] = 0X03;     // 功能码
    BUFF[_cnt++] = 0x07;     // 数据长度
    BUFF[_cnt++] = BYTE0(A); // 数据内容,小段模式，低位在前
    BUFF[_cnt++] = BYTE1(A); // 需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++] = BYTE0(B);
    BUFF[_cnt++] = BYTE1(B);
    BUFF[_cnt++] = BYTE0(C);
    BUFF[_cnt++] = BYTE1(C);
    // BUFF[_cnt++] = fu;
    BUFF[_cnt++] = 1;

    // SC和AC的校验直接抄最上面上面简介的即可
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    // for(i=0;i<_cnt;i++) UsartSendByte(USART2,BUFF[i]);//串口逐个发送数据
    for (i = 0; i < _cnt; i++)
    {
        // USART_SendData(USART1, BUFF[i]);
        Serial_SendByte(BUFF[i]); // usart1
    }
}

/*发送姿态四元数数据*/
void anotc_sendquat(uint16_t A, uint16_t B, uint16_t C, uint16_t D)
{
    int i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt = 0;
    BUFF[_cnt++] = 0xAA;     // 帧头
    BUFF[_cnt++] = 0xFF;     // 目标地址
    BUFF[_cnt++] = 0X04;     // 功能码
    BUFF[_cnt++] = 0x09;     // 数据长度
    BUFF[_cnt++] = BYTE0(A); // 数据内容,小段模式，低位在前
    BUFF[_cnt++] = BYTE1(A); // 需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++] = BYTE0(B);
    BUFF[_cnt++] = BYTE1(B);
    BUFF[_cnt++] = BYTE0(C);
    BUFF[_cnt++] = BYTE1(C);
    BUFF[_cnt++] = BYTE0(D);
    BUFF[_cnt++] = BYTE1(D);
    // BUFF[_cnt++] = fu;
    BUFF[_cnt++] = 1;

    // SC和AC的校验直接抄最上面上面简介的即可
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    // for(i=0;i<_cnt;i++) UsartSendByte(USART2,BUFF[i]);//串口逐个发送数据
    for (i = 0; i < _cnt; i++)
    {
        // USART_SendData(USART1, BUFF[i]);
        Serial_SendByte(BUFF[i]); // usart1
    }
}
