#ifndef __TIMER_H
#define __TIMER_H
// void Timer_Init(void);
uint8_t OC1Get_Duty(void);
uint8_t OC2Get_Duty(void);
uint8_t OC3Get_Duty(void);
uint8_t OC4Get_Duty(void);
void IC_Init(void);
void Receiver_display();
void Receiver_displaytarget();
void Target_Read_Receiver(float *target_x, float *target_y, float *target_z, float *Thro);


#endif
