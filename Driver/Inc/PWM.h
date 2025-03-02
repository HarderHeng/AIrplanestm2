#ifndef __PWM_H
#define __PWM_H
void PWM_Init(void);
void PWM_SetCompare1(uint16_t compare); // 设置pwm通道1占空比
void PWM_SetCompare2(uint16_t compare); // 设置pwm通道2占空比
void PWM_SetCompare3(uint16_t compare); // 设置pwm通道3占空比
void PWM_SetCompare4(uint16_t compare); // 设置pwm通道4占空比
void PWM_Start(void);
void PWM_Set(int M1, int M2, int M3, int M4);
void PWM_fromReceiver(void);

#endif