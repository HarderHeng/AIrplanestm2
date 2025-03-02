#ifndef _TASK_ATTITUDE_H
#define _TASK_ATTITUDE_H
#include <stdlib.h>

typedef struct MPU6050Data
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} MPU6050Data;

extern MPU6050Data attitude;

// typedef struct HMCdata{
//     int16_t x;
//     int16_t y;
//     int16_t z;
// };
// extern HMCdata hmc;
extern void TASK_Attitude(void *p_arg);
extern void TASK_Pid(void *p_arg);

extern float angle_pitch;
extern float angle_roll;
extern float angle_yaw;
#endif
