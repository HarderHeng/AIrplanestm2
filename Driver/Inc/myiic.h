#ifndef _MYIIC_H
#define _MYIIC_H

#include <stdint.h>
#define IIC_GPIO_PORT GPIOB
#define IIC_SCL_GPIO_PIM 8
#define IIC_SDA_GPIO_PIM 9

// 读取iic的数据读取状态
void getIICstatus();
// 清除iic的数据状态
void clearIICstatus();

// iic操作函数
void IIC_Init(void);  // 初始化iic的io口
void IIC_Start(void); // 发送iic开始信号
void IIC_Stop(void);  // 发送iic停止信号

void IIC_SendByte(uint8_t byte); // iic发送一个字节数据
uint8_t IIC_ReceiveByte(void);   // iic接收一个字节数据

void IIC_SendACK(void); // iic发送ack信号
void IIC_NACK(void);            // iic不发送ack信号
uint8_t IIC_WaitACK(void);      // iic等待ACK信号
uint8_t IIC_ReceiveACK(void);   // iic接收应答

uint8_t iic_send_bytes(uint8_t *buffer, int size); /* 发送多个字节 */
void iic_read_bytes(uint8_t *buffer, int size);    /* 读取多个字节 */

#endif
