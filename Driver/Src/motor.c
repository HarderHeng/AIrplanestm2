#include <stdint.h>
#include "includes.h"
#include "task.h" //test

int Motor[4];

typedef struct axis_val
{
    double err_k;
    double err_k_1;
    double err_k_2;
} axis_val_t;

typedef struct pid_val
{
    axis_val_t x_axis;
    axis_val_t y_axis;
    axis_val_t z_axis;
} pid_val_t;

float cast_to_change(float val, float begin, float end)
{
    // return ((val-1000) * (end - begin))/ 1000 + begin;
    if (val < begin)
    {
        return begin;
    }
    else if (val > end)
    {
        return end;
    }
    else
    {
        return val;
    }
}

// void angle_pid(void *args){
//     while(1){
//         CtrlAngle();
//         Delay_ms(20);
//     }
// }

/**
 * 横滚：roll：绕x轴，正方向14+，23-
 * 俯仰：pitch：绕y轴，正方向12+，34-
 * 偏航：yaw：绕z轴
 */

/**
 * @brief 映射pid的返回值和电机的转速
 */
void rate_pid(void)
{
    float partial = 0.8;

    /*获取接收机的三个角度和油门占比:test时暂时关闭*/
    // Target_Read_Receiver(&target_x, &target_y, &target_z, &thro);
    // thro = thro * 800 + 1050;
    thro = 1400; // test
    CtrlAngle();
    CtrlRate();
    // madgwick_printangle(&roll, &pitch, &yaw);
    // pid_display(roll, pitch, yaw);
    // 两环pid之后返回pitch、roll、yaw,也就是pid反馈控制得到的值提供给电机

    // 打印遥控器的值
    // 将输出值融合到四个电机
    if (thro < 1050) // 小于预计的油门
    {
        pitch = 0, roll = 0, yaw = 0;
    }
    else
    {
        pitch = cast_to_change(pitch, -(thro - 1000) * partial, (thro - 1000) * partial); // 这里做了一个限定,将变化的范围定在
        roll = cast_to_change(roll, -(thro - 1000) * partial, (thro - 1000) * partial);
        yaw = cast_to_change(yaw, -(thro - 1000) * partial, (thro - 1000) * partial);
    }
    // madgwick_printangle(&roll, &pitch, &thro);
    // Motor[2] = (int16_t)cast_to_change((thro - pitch - roll - yaw), 1000, 1800); // M3
    // Motor[1] = (int16_t)cast_to_change((thro + pitch - roll + yaw), 1000, 1800); // M2
    // Motor[3] = (int16_t)cast_to_change((thro - pitch + roll + yaw), 1000, 1800); // M4
    // Motor[0] = (int16_t)cast_to_change((thro + pitch + roll - yaw), 1000, 1800); // M1

    Motor[2] = (int16_t)cast_to_change((thro - pitch - roll), 1100, 1800); // M3
    Motor[1] = (int16_t)cast_to_change((thro + pitch - roll), 1100, 1800); // M2
    Motor[3] = (int16_t)cast_to_change((thro - pitch + roll), 1100, 1800); // M4
    Motor[0] = (int16_t)cast_to_change((thro + pitch + roll), 1100, 1800); // M1
    // M_display(Motor[0], Motor[1], Motor[2], Motor[3]);
    PWM_Set(Motor[0], Motor[1], Motor[2], Motor[3]);
}
void M_display(int M1, int M2, int M3, int M4)
{
    char ch1[10];
    char ch2[10];
    char ch3[10];
    char ch4[10];

    float m1, m2, m3, m4 = 0;
    m1 = M1;
    m2 = M2;
    m3 = M3;
    m4 = M4;

    float_to_string(m1, ch1, 0);
    float_to_string(m2, ch2, 0);
    float_to_string(m3, ch3, 0);
    float_to_string(m4, ch4, 0);
    USART1_printf("M1:Z:%s ", ch1);
    USART1_printf("M2:Y:%s ", ch2);
    USART1_printf("M3:X:%s ", ch3);
    USART1_printf("M4:%s\n", ch4);
}
