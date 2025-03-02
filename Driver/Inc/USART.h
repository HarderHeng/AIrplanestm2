#ifndef __USART_H
#define __USARR_H
void Serial_Init(void);
void Serial_SendByte(uint8_t byte);
uint8_t Serial_ReceiveByte(void);
uint8_t GetRxFlag(void);
void USART_IRQHandler(void);
void Serial_SendString(char *String);
void USART1_printf(char *fmt, ...);
void float_to_string(float number, char *str, int decimal_places);
#endif
