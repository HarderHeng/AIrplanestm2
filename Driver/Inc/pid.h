#ifndef _PID_H
#define _PID_H

extern float pitch; // 输出的角速度
extern float roll;  // 输出的角速度
extern float yaw;   // 输出的角速度
extern float thro;
extern float target_x;
extern float target_y;
extern float target_z;

void CtrlRate(void);
void CtrlAngle(void);
// void Cal_PID_Position(PID, float target, float measure, int32_t dertT);
void Init_PID();
void pid_display(float roll, float pitch, float yaw);

#endif